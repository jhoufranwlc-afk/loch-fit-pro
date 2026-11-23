# LOCH Fit PRO

App web instalable (PWA) de entrenamiento: temporizador por rondas, rutinas, figuras 2D animadas de cada ejercicio, voz, historial y programas de varias semanas. Funciona sin conexión.

## Funciones
- Temporizador rápido y rutinas con calentamiento y estiramiento guiados
- 47 ejercicios con figura animada, consejos de técnica y zona trabajada iluminada
- Voz en español, historial, racha y meta semanal
- Registro de peso y repeticiones con récords
- Paquetes de Boxeo y Calistenia
- Programas de 4 semanas (Calistenia, Boxeo, Mancuernas)
- Celebración al terminar y logros (récord, racha, meta semanal, programa)
- Respaldo en archivo JSON

## Uso
Abre `index.html` desde un servidor local, por ejemplo:

```
python3 -m http.server 8000
```

y entra a `http://localhost:8000`. En el iPhone: Safari → Compartir → Añadir a pantalla de inicio.

Los datos se guardan solo en el navegador del dispositivo.

## Archivos
- `index.html`: la app
- `figures.js`: figuras y animaciones
- `sw.js` y `manifest.webmanifest`: modo sin conexión e instalación
- `fonts/`, `icon-*.png`, `apple-touch-icon.png`: recursos
