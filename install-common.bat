@echo off
setlocal

set repository_url=https://github.com/Galaxia5987/Common.git

set destination_directory=src\main\java\frc\robot

cd "%destination_directory%"

if not exist "common" (
    git clone %repository_url%
)

cd common
git pull origin
git checkout 2024-common

cd ..
cd ..
cd ..
cd ..
cd ..
cd test\java\frc\robot

if not exist "common" (
    mkdir common
)

endlocal