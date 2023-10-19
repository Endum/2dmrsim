@echo off

call mvn clean package

IF ERRORLEVEL 1 goto TheEnd

copy io.github.ericmedvet.mrsim2d.core\target\mrsim2d.core-1.0.0.jar ..\2d-robot-evolution\io.github.ericmedvet.robotevo2d.assembly\target\robotevo2d.assembly-bin\modules\mrsim2d.core-0.8.6.jar

:TheEnd