
PhysX Wrapper

Copyright 2015 University of Central Florida



SUMMARY
=======

This library wraps up the native calls to NVIDIA's PhysX API and
provides higher-level functions to C# for use.  It is not a one-to-one
mapping but rather provides a higher-level interface (this allows both
the use of .NET and Mono).

This code has been released under the Apache License Version 2.0 (see 
licensing information below) and is provided by the University of
Central Florida and the U.S. Army's Army Research Laboratory Simulation
and Training Technology Center (under the Military Open Simulator
Enterprise Strategy, or MOSES, program).  Individual contributors can
be found in the CONTRIBUTORS.txt file included in this archive.


COMPILING
=========

This wrapper has a number of dependencies:

- NVIDIA's PhysX (version 3.3.3 or later)
- NVIDIA's Cuda library (for GPU support)
- Pthread library
- "msinttypes" package (for Windows only; see code.google.com/p/msinttypes)
- UCF IST's ATLAS library (see www.irl.ucf.edu)

This library uses the scons build tool (www.scons.org).

To compile:

LINUX:
scons physxPath=<path_to_PhysXSDK_root> pthreadPath=<path_to_pthread_root> atlasPath=<path_to_ATLAS_root> cudaPath=<path_to_CUDA_SDK_root>

example:
scons physXPath=/usr/local/physx-3.3.3-1.3.3-src/PhysX-3.3/PhysXSDK pthreadPath=/usr atlasPath=/usr/local/atlas cudaPath=/usr/local/cuda-7.0

WINDOWS:
scons physxPath=<path_to_PhysXSDK_root> pthreadPath=<path_to_pthread_root> atlasPath=<path_to_ATLAS_root> cudaPath=<path_to_CUDA_SDK_root> msinttypesPath=<path_to_msinttypes_root>

example:
scons physXPath=C:\Program Files\NVIDIA\PhysX-3.3.3\PhysXSDK pthreadPath=C:\pthread-win32-2.8.0 atlasPath=C:\Program Files\UCF IST\ATLAS cudaPath=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v7.5 msinttypesPath=C:\msinttypes-r26


LICENSE
=======

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


VERSIONS
========

1.0.0   Initial version.

