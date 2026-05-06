#!/usr/bin/env python3
import argparse
import html
import json
import re
import threading
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import asyncio
import sys
import traceback

try:
    import zendriver as zd
    from zendriver import cdp
except Exception as e:
    sys.stderr.write("IMPORT_ERROR:" + str(e) + "\n")
    sys.exit(1)

class ZendriverDaemon:
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.browser = None
        self.fetch_lock = None
        self._start_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._start_thread.start()
        # initialize browser in the loop
        fut = asyncio.run_coroutine_threadsafe(self._start_browser(), self.loop)
        try:
            fut.result(timeout=30)
        except Exception as e:
            sys.stderr.write("BROWSER_START_ERROR:" + str(e) + "\n")

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def _start_browser(self):
        try:
            self.browser = await zd.start()
            if self.fetch_lock is None:
                self.fetch_lock = asyncio.Lock()
        except Exception as e:
            raise

    @staticmethod
    def _looks_like_json(text):
        stripped = (text or "").lstrip()
        return stripped.startswith("{") or stripped.startswith("[")

    @staticmethod
    def _is_cf_challenge(text):
        lowered = (text or "").lower()
        return (
            "just a moment" in lowered
            or "_cf_chl_opt" in lowered
            or "enable javascript and cookies" in lowered
            or "challenge-platform" in lowered
        )

    @staticmethod
    def _content_text(content):
        if not content:
            return ""
        match = re.search(r"<pre[^>]*>(.*?)</pre>", content, re.IGNORECASE | re.DOTALL)
        if match:
            return html.unescape(match.group(1)).strip()
        return content.strip()

    async def _navigate(self, page, url):
        try:
            await page.send(cdp.page.navigate(url))
        except Exception:
            try:
                await asyncio.wait_for(page.get(url), timeout=20)
            except Exception:
                pass

    async def _browser_fetch(self, page, url):
        script = """
        (async (target) => {
          const response = await fetch(target, {
            method: 'GET',
            credentials: 'include',
            cache: 'no-store',
            headers: {
              'Accept': 'application/json,text/plain,*/*',
              'X-Requested-With': 'XMLHttpRequest'
            }
          });
          const text = await response.text();
          return JSON.stringify({
            status: response.status,
            contentType: response.headers.get('content-type') || '',
            text
          });
        })(%s)
        """ % json.dumps(url)
        result = await asyncio.wait_for(
            page.evaluate(script, await_promise=True, return_by_value=True),
            timeout=30,
        )
        if not isinstance(result, str):
            return ""
        data = json.loads(result)
        return (data.get("text") or "").strip()

    async def fetch(self, url):
        if not self.browser:
            await self._start_browser()
        if self.fetch_lock is None:
            self.fetch_lock = asyncio.Lock()

        async with self.fetch_lock:
            try:
                page = self.browser.tabs[0]
            except Exception:
                page = await self.browser.get("about:blank")

            await self._navigate(page, url)

            last_text = ""
            deadline = time.monotonic() + 55
            while time.monotonic() < deadline:
                try:
                    try:
                        await page.wait_for_ready_state("complete", timeout=5)
                    except Exception:
                        pass

                    text = self._content_text(await page.get_content())
                    if text:
                        last_text = text
                    if self._looks_like_json(text):
                        return text

                    if self._is_cf_challenge(text):
                        try:
                            await asyncio.wait_for(page.verify_cf(timeout=10), timeout=15)
                        except Exception:
                            pass
                        await asyncio.sleep(1)
                        continue

                    fetched = await self._browser_fetch(page, url)
                    if fetched:
                        return fetched
                except Exception:
                    await asyncio.sleep(1)

            try:
                fetched = await self._browser_fetch(page, url)
                if fetched:
                    return fetched
            except Exception:
                pass

            return last_text

daemon = ZendriverDaemon()

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/health":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
            return
        # support /fetch?url=...
        if self.path.startswith("/fetch"):
            from urllib.parse import urlparse, parse_qs
            parsed = urlparse(self.path)
            qs = parse_qs(parsed.query)
            if 'url' in qs:
                url = qs['url'][0]
                try:
                    fut = asyncio.run_coroutine_threadsafe(daemon.fetch(url), daemon.loop)
                    content = fut.result(timeout=60)
                    if content is None:
                        content = ''
                    if isinstance(content, str):
                        content_bytes = content.encode('utf-8', errors='replace')
                    else:
                        content_bytes = content
                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write(content_bytes)
                    return
                except Exception as e:
                    tb = traceback.format_exc()
                    self.send_response(500)
                    self.end_headers()
                    self.wfile.write(("ERROR:" + str(e) + "\n" + tb).encode('utf-8', errors='replace'))
                    return
            else:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b'Missing url parameter')
                return
        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        if self.path == "/fetch":
            content_length = int(self.headers.get('Content-Length',0))
            post = self.rfile.read(content_length)
            try:
                data = json.loads(post.decode('utf-8'))
                url = data.get('url','')
            except Exception:
                s = post.decode('utf-8')
                from urllib.parse import parse_qs
                qs = parse_qs(s)
                url = qs.get('url',[''])[0]
            if not url:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b'Missing url')
                return
            try:
                fut = asyncio.run_coroutine_threadsafe(daemon.fetch(url), daemon.loop)
                content = fut.result(timeout=60)
                if content is None:
                    content = ''
                if isinstance(content, str):
                    content_bytes = content.encode('utf-8', errors='replace')
                else:
                    content_bytes = content
                self.send_response(200)
                self.end_headers()
                self.wfile.write(content_bytes)
                return
            except Exception as e:
                tb = traceback.format_exc()
                self.send_response(500)
                self.end_headers()
                self.wfile.write(("ERROR:" + str(e) + "\n" + tb).encode('utf-8', errors='replace'))
                return
        self.send_response(404)
        self.end_headers()

def run_server(host='127.0.0.1', port=37337):
    server = ThreadingHTTPServer((host, port), Handler)
    print("Zendriver daemon listening on %s:%d" % (host, port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=37337)
    args = parser.parse_args()
    run_server('127.0.0.1', args.port)
