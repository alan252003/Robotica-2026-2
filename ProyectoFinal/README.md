# Proyecto final de robótica (dos manipuladores)
En este directorio se encontrarán los archivos necesarios para simular el movimiento de dos manipuladores, uno de tres grados de libertad en el plano XY y otro de dos grados de libertad en el espacio. De forma que mediante la interfaz gráfica de rviz de pueda controlar su movimiento por separado.

## Instrucciones de uso:
- Compilar el repositorio desde "manipulators_ws"
- Desplegar el programa mediante el archivo .launch en el directorio ./src/manipulators_launcher/launch/
- Una vez se despliegue RVIZ, esperar a que los controladores de los manipuladores se carguen
- En la sección de Panels/Tool properties/Publis Point, cambiar el nombre del tópico /clicked_point por /clicked_point_xy para el manipulador planar y /clicked_point_zy para el manipulador espacial para seleccionar que manipulador mover
- Una vez seleccionado el manipulador, habilitar Publish Point y seleccionar un punto de los planos correspondientes

*NOTA: para el segundo manipulador en ciertos puntos no se filtran bien las posiciones de los eslabones por lo que requiere mejora.
