@echo off
echo Check for Node.js...
node -v
if %errorlevel% neq 0 (
    echo [ERROR] Node.js is NOT installed. Please install it from https://nodejs.org/
    pause
    exit /b
)

echo Installing dependencies...
call npm install

echo Initializing Android Platform...
call npx cap add android

echo Syncing web assets...
call npx cap sync

echo Setup Complete!
echo You can now open the project in Android Studio with: npx cap open android
pause
