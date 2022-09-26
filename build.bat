
del /Q /F build\Release\CMakeCache.txt

cmake -S . -B build\Release -G "Ninja" ^
-DCMAKE_BUILD_TYPE=Release ^
-DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=TRUE ^
-DCMAKE_EXPORT_COMPILE_COMMANDS=ON ^
-DCMAKE_C_COMPILER=clang-cl ^
-DCMAKE_CXX_COMPILER=clang-cl

cmake --build build\Release
