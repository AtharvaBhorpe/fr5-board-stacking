const CACHE = "workcell-shell-v3";
self.addEventListener("install", (event) => {
  event.waitUntil(
    (async () => {
      const cache = await caches.open(CACHE);
      const response = await fetch("/");
      const html = await response.clone().text();
      const assets = [
        ...html.matchAll(/(?:src|href)="(\/assets\/[^\"]+)"/g),
      ].map((match) => match[1]);
      await cache.put("/", response);
      await cache.addAll([
        ...assets,
        "/manifest.webmanifest",
        "/icon-192.png",
        "/icon-512.png",
        "/recorder.js",
      ]);
    })(),
  );
  self.skipWaiting();
});
self.addEventListener("activate", (event) =>
  event.waitUntil(
    caches
      .keys()
      .then((keys) =>
        Promise.all(
          keys.filter((k) => k !== CACHE).map((k) => caches.delete(k)),
        ),
      )
      .then(() => self.clients.claim()),
  ),
);
self.addEventListener("fetch", (event) => {
  const url = new URL(event.request.url);
  if (
    event.request.method !== "GET" ||
    url.origin !== location.origin ||
    url.pathname.startsWith("/api/") ||
    url.pathname === "/workcell-ca.crt"
  )
    return;
  event.respondWith(
    fetch(event.request)
      .then((response) => {
        if (response.ok) {
          const copy = response.clone();
          caches.open(CACHE).then((cache) => cache.put(event.request, copy));
        }
        return response;
      })
      .catch(() => caches.match(event.request)),
  );
});
