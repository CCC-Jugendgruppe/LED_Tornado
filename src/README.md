# Probleme:
* `Could not determine GDB version using command: arm-none-eabi-gdb --version arm-none-eabi-gdb: error while loading shared libraries: libncurses.so.5: cannot open shared object file: No such file or directory` fehler-Meldung
	* Fix: `arm-none-eabi-gcc` und `arm-none-eabi-newlib` installieren und in der CubeIDE unter Window->Preferences->Toolchain Manager eine neue hinzufügen, mit dem Prefix /usr/bin. 
	* 
