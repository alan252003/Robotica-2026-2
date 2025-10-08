#!/bin/bash
echo "Archivo ejecutable inicializado"
cd ${HOME}
DirExist="~/Practica1"
if [ -d "$DirExist" ]; then
	echo "El directorio Practica1 ya existe."
else
	mkdir -p Practica1
	echo "Se ha creado el directorio Practica1"
fi
cd ./Practica1
mkdir -p Letras/Integrantes
cd Letras
touch {a.txt,b.txt,c.txt}
cd Integrantes
touch AlanRogelioCorreaPalacios.txt
tree ${HOME}/Practica1
rm -r ${HOME}/Practica1/*
if ! [ -d "$DirExist" ]; then
	echo "El directorio Practica1 se ha eliminado correctamente."
else
	echo "ERROR: No se ha podido borrar le directorio."
fi
