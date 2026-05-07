# ArUco Detector — Pendientes y mejoras

Lista viva de tareas pendientes para el detector ArUco. Tickear cuando se completen.

## Pendientes funcionales

- [ ] **Kalman filter para posiciones ArUco**. BlobDetection suaviza con `_kalmanFilterRobots[][]` y `_dirFilteRobots[][]`. ArucoDetection hoy entrega posiciones crudas frame-a-frame, así que el downstream verá más jitter.
- [ ] **Predicción de pelota cuando no se detecta**. BlobDetection extrapola con velocidad estimada cuando el blob naranja se pierde (ver `findBall` en `PositionProcessing.cpp`). ArUco hoy solo deja la pelota `outdated`.
- [ ] **Calibrador HSV para la pelota desde la UI**. Hoy el rango HSV vive en `Config/ArucoConfig.json` y hay que editarlo a mano. Una idea: trackbars OpenCV en una ventana aparte (similar a `tests/hsv_calibrator.py` del proyecto Software_VSSS).
- [ ] **Persistir el toggle "Use ArUco" entre sesiones**. Hoy arranca siempre en `false`; agregar a `Config/CameraConfigL.json` o crear `Config/DetectorConfig.json`.
- [ ] **Soporte para Spinnaker en Docker/devcontainer**. El SDK no se incluye en `Dockerfile` ni `.devcontainer/Dockerfile` — requiere instalación manual en `/opt/spinnaker/`.
- [ ] **Control de parámetros Spinnaker (gain, exposure)**. La UI solo configura cámaras USB vía `v4l2-ctl`. Para Spinnaker hay que usar la API GenAPI.
- [ ] **Aplicar `WarpCorrection` validado para el branch ArUco**. Hoy `setFrame()` aplica corrección antes de pasar a ArUco si el toggle de corrección está activo, pero falta verificar en cancha real que la posición resultante en cm es correcta.
- [ ] **Mensajes de error más descriptivos**. Si Spinnaker o ArUco fallan, el usuario ve "Problem trying to open the camera!" — habría que distinguir.

## Limpieza/ergonomía

- [ ] **Diálogo de configuración batch de IDs ArUco**. Hoy hay que clickear el ⚙ de cada `RobotWidget` uno por uno. Idea: un diálogo único con 6 spinboxes en `Configure` menu.
- [ ] **Mostrar el ID detectado encima del robot en el debug frame**. `cv::aruco::drawDetectedMarkers` ya escribe el ID, pero podríamos mostrar también el slot (ROBOT1, ADV1, etc.).
- [ ] **Migrar a la API nueva `cv::aruco::ArucoDetector`** cuando se actualice OpenCV a 4.7+. La actual `cv::aruco::detectMarkers` está deprecada en versiones nuevas pero funciona con OpenCV 4.5.4.
- [ ] **Tests unitarios**. No hay tests en `vsss-vision`. Mínimo: un test que cargue una imagen sintética con un marker conocido y verifique que `runFromFrame` lo detecta.

## Conocidos / no tocar

- Los botones ⚙ de `RobotWidget` para los slots 4..20 no hacen nada útil (sólo 0..5 están mapeados a entities). El código retorna early si `m_index >= kEntityCount`. No es un bug — es deliberado.
- BlobDetection se conserva intacto. El toggle del menú permite alternar — no se borra el detector legacy.
- El TBB flow graph y CameraManager no se tocaron. Cualquier regresión ahí no es de la integración ArUco.
