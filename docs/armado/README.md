# Diagramas de armado por piso

En orden:

1. `1_base.png`: estructura de la Base, motores, ruedas y separadores.
2. `2_medio_nano_encoders.png`: Arduino Nano en la protoboard mini y los encoders; prueba de cuentas sin batería.
3. `3_medio_l298n_motores.png`: L298N, cables de los motores y control desde el Nano.
4. `4_medio_alimentacion.png`: batería, interruptor, regulador y L298N; prueba de las ruedas en el aire.
5. `5_tope_pi_lidar.png`: Raspberry Pi, LiDAR, IMU y cables USB.

Se generan con `docs/figuras_tomo/fuentes/armado/` usando los dibujos de los componentes de Fritzing (paquete `fritzing-parts`) y los del material del curso (L298N y XT60):

```bash
export FZ_PARTS=<fritzing-parts>/svg/core/breadboard CURSO_PARTS=<carpeta del .fzz del curso, descomprimido>
cd docs/figuras_tomo/fuentes/armado
python3 base.py 1.svg ../../../../src/bumperbot_description/meshes/base_link.STL
python3 medio_a.py 2.svg && python3 medio_b.py 3.svg && python3 medio_c.py 4.svg && python3 tope.py 5.svg
```
