@rem #############################################################################
@rem #
@rem # ViSP, open source Visual Servoing Platform software.
@rem # Copyright (C) 2005 - 2019 by Inria. All rights reserved.
@rem #
@rem # This software is free software; you can redistribute it and/or modify
@rem # it under the terms of the GNU General Public License as published by
@rem # the Free Software Foundation; either version 2 of the License, or
@rem # (at your option) any later version.
@rem # See the file LICENSE.txt at the root directory of this source
@rem # distribution for additional information about the GNU GPL.
@rem #
@rem # For using ViSP with software that can not be combined with the GNU
@rem # GPL, please contact Inria about acquiring a ViSP Professional
@rem # Edition License.
@rem #
@rem # See http://visp.inria.fr for more information.
@rem #
@rem # This software was developed at:
@rem # Inria Rennes - Bretagne Atlantique
@rem # Campus Universitaire de Beaulieu
@rem # 35042 Rennes Cedex
@rem # France
@rem #
@rem # If you have questions regarding the use of this file, please contact
@rem # Inria at visp@inria.fr
@rem #
@rem # This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
@rem # WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
@rem #
@rem # Description:
@rem # visp-config.bat script for Windows.
@rem # Auto-generated from visp-config.bat.in by cmake.
@rem #
@rem # Authors:
@rem # Fabien Spindler
@rem #
@rem #############################################################################


@echo off

@rem Get the dirname
@rem for /F %%x in ('CHDIR') do set PREFIX=%%x\..
set CURPATH=%CD /d %%~dp0%

@rem echo CURPATH: %CURPATH%

set PREFIX=%CURPATH%\..

set VERSION=3.2.1

set DEFS=-std=c++11, -openmp

set INCLUDE="%PREFIX%/include"

set OPENMP_SUPPORT=yes

set LIBDIR="%PREFIX%///lib;D:/visp-ws/opencv-4.1.1/build/x64/vc15/lib;C:/Program Files (x86)/Microsoft SDKs/Windows/v7.1A/Lib/x64;C:/Program Files/Basler/pylon 5/Development/lib/x64;C:/Program Files (x86)/Intel RealSense SDK 2.0 (Win7)/lib/x64"

set LIBS_DEBUG=visp_core321d.lib;visp_gui321d.lib;visp_imgproc321d.lib;visp_io321d.lib;d.lib;visp_klt321d.lib;visp_me321d.lib;visp_sensor321d.lib;visp_ar321d.lib;visp_blob321d.lib;visp_robot321d.lib;visp_visual_features321d.lib;visp_vs321d.lib;visp_vision321d.lib;visp_detection321d.lib;visp_mbt321d.lib;visp_tt321d.lib;visp_tt_mi321d.lib;opencv_world411d.lib;Gdi32.Lib;PylonBase_MD_VC120_v5_0.lib;GCBase_MD_VC120_v3_0_Basler_pylon_v5_0.lib;GenApi_MD_VC120_v3_0_Basler_pylon_v5_0.lib;PylonUtility_MD_VC120_v5_0.lib;realsense2.lib

set LIBS_OPTIMIZED=visp_core321.lib;visp_gui321.lib;visp_imgproc321.lib;visp_io321.lib;.lib;visp_klt321.lib;visp_me321.lib;visp_sensor321.lib;visp_ar321.lib;visp_blob321.lib;visp_robot321.lib;visp_visual_features321.lib;visp_vs321.lib;visp_vision321.lib;visp_detection321.lib;visp_mbt321.lib;visp_tt321.lib;visp_tt_mi321.lib;opencv_world411.lib;Gdi32.Lib;PylonBase_MD_VC120_v5_0.lib;GCBase_MD_VC120_v3_0_Basler_pylon_v5_0.lib;GenApi_MD_VC120_v3_0_Basler_pylon_v5_0.lib;PylonUtility_MD_VC120_v5_0.lib;realsense2.lib

@rem Test if an argument is provided

if "%1" == "" goto USAGE

@rem Parse the argument list
for %%a in (%*) do (
@rem  echo Read arg %%a
  if "%%a" == "--help" (
	goto USAGE
  )
  if "%%a" == "--prefix" (
	echo %PREFIX%
	goto END
  )
  if "%%a" == "--def" (
	echo %DEFS%
	goto END
  )
  if "%%a" == "--include" (
	echo %INCLUDE%
	goto END
  )
  if "%%a" == "--openmp" (
	echo %OPENMP_SUPPORT%
	goto END
  )
  if "%%a" == "--libpath" (
	echo %LIBDIR%
	goto END
  )
  if "%%a" == "--libs-debug" (
	echo %LIBS_DEBUG%
	goto END
  )
  if "%%a" == "--libs-optimized" (
	echo %LIBS_OPTIMIZED%
	goto END
  )
  if "%%a" == "--version" (
	echo ViSP %VERSION% Visual Servoing Platform
	echo.
	echo Copyright 2005 - 2019 Inria. All rights reserved.
	goto END
  )
  if "%%a" == "--dumpversion" (
	echo %VERSION%
	goto END
  )

)

:USAGE
echo ViSP %VERSION% (Visual Servoing Platform)
echo Copyright (C) 2005 - 2019 Inria. All rights reserved.
echo.
echo Usage: %0 [--prefix] [--def] [--include] [--openmp] [--libpath] 
echo [--libs-debug] [--libs-optimized] [--version] [--dumpversion] [--help]
echo.
echo  --prefix          Show ViSP installation prefix.
echo  --def             Print pre-processor definitions.
echo  --include         Print include directories.
echo  --openmp          Indicates if OpenMP support has to be turned on/off.
echo  --libpath         Print library directories.
echo  --libs-debug      Print library dependencies for debug configuration.
echo  --libs-optimized  Print library dependencies for optimized configuration.
echo  --dumpversion     Output ViSP version information.
echo  --help            Display this help and exit.
echo.
goto END

:End
