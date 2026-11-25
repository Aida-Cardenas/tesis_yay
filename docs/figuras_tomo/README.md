# Figuras del tomo

| Archivo | Figura del tomo |
|---|---|
| `figura_modelo_cad.png` | Vistas del modelo CAD de las tres placas y del robot ensamblado (III.2) |
| `figura_diagrama_conexiones.png` | Diagrama de conexiones (III.3, Conexiones eléctricas) |
| `figura_arquitectura_general.png` | Diagrama de la arquitectura general (III.4) |
| `figura_camino_del_comando.png` | Diagrama del camino del comando (III.4.3) |
| `figura_arquitectura_gemelo.png` | Arquitectura del gemelo digital (III.4.4) |

Los diagramas se generan con los scripts de `fuentes/` (SVG a PNG con `rsvg-convert -z 1.5`).
Las vistas del modelo salen de las mallas de `bumperbot_description/meshes`, colocadas como en el URDF
(`build_glb.py`) y dibujadas con three.js en Chromium (`index.html`, `shot.js`, `compose.py`).
