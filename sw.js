// Guarda la app (y su tipografía) en el teléfono para que abra sin internet.
// Si cambias algún archivo, sube el número de versión (v1 -> v2) para que se actualice.
var CACHE = 'loch-fit-pro-v14';
var FILES = [
  './', 'index.html', 'figures.js', 'manifest.webmanifest',
  'icon-192.png', 'icon-512.png', 'apple-touch-icon.png',
  'fonts/barlow-condensed-600.woff2', 'fonts/barlow-condensed-700.woff2', 'fonts/barlow-condensed-800.woff2'
];

self.addEventListener('install', function (e) {
  e.waitUntil(caches.open(CACHE).then(function (c) { return c.addAll(FILES); }));
  self.skipWaiting();
});

self.addEventListener('activate', function (e) {
  e.waitUntil(
    caches.keys().then(function (keys) {
      return Promise.all(keys.filter(function (k) { return k !== CACHE; }).map(function (k) { return caches.delete(k); }));
    })
  );
  self.clients.claim();
});

self.addEventListener('fetch', function (e) {
  if (e.request.method !== 'GET') return;
  var range = e.request.headers.get('range');
  if (range && /\.mp4$/.test(new URL(e.request.url).pathname)) {
    // Safari necesita respuestas parciales (Range) para reproducir video desde la caché.
    e.respondWith(
      caches.match(e.request.url).then(function (hit) { return hit || fetch(e.request.url); })
        .then(function (res) { return res.arrayBuffer().then(function (buf) {
          var m = /bytes=(\d*)-(\d*)/.exec(range), a = m && m[1] ? +m[1] : 0, b = m && m[2] ? +m[2] : buf.byteLength - 1;
          return new Response(buf.slice(a, b + 1), { status: 206, headers: {
            'Content-Type': 'video/mp4', 'Content-Range': 'bytes ' + a + '-' + b + '/' + buf.byteLength, 'Content-Length': String(b - a + 1) } });
        }); })
    );
    return;
  }
  e.respondWith(
    caches.match(e.request).then(function (hit) { return hit || fetch(e.request); })
  );
});
