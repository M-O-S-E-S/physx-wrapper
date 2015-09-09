
# Import sys so we call exit() if we need to
import sys


# Figure out whether we're on a 32-bit or 64-bit system
import struct
arch = str(struct.calcsize("P") * 8) + 'bit'

# Get the build target (e.g., win32, posix, etc.)
buildTarget = ARGUMENTS.get('platform', str(Platform()))

# If the buildTarget isn't for mobile (not "ios" or "android") then add
# bit info (to support 32-bit or 64-bit builds)
if buildTarget != 'ios' and buildTarget != 'android':
   buildTarget = buildTarget + '.' + arch

# Echo for user
print 'Building for ' + buildTarget + '...'

###############################################################################

# Get whether this program is compiling in debug or release mode, debug should
# be set to 0 when in release and anything else in debug
debug = ARGUMENTS.get('debug', 0)

###############################################################################

# Base paths for external libraries (platform dependent)
if buildTarget == 'win64.64bit':
   # PhysX
   physxPath = 'W:/physX/PhysX-3.3.3/PhysXSDK';
   # pthread 
   pthreadPath = 'L:/pthread-win32-2.8.0'
   # ATLAS
   atlasPath = 'W:/atlas'
   # inttypes.h for MSVC
   msinttypesPath = 'L:/msinttypes-r26'
elif buildTarget == 'win32.32bit':
   # PhysX
   physxPath = 'W:/physX/PhysX-3.3.3/PhysXSDK';
   # pthread 
   pthreadPath = 'L:/pthread-win32-2.8.0'
   # ATLAS
   atlasPath = 'W:/atlas'
   # inttypes.h for MSVC
   msinttypesPath = 'L:/msinttypes-r26'
elif buildTarget == 'posix.64bit':
   # PhysX
   physxPath = '/irl/tools/libs/physx-3.3.3-1.3.3-src/PhysX-3.3/PhysXSDK';
   # pthread (see subpaths below)
   pthreadPath = '/usr'
   # ATLAS
   atlasPath = '#../atlas'
else:
   # Unsupported architecture so bail
   print "Unsupported target type ", buildTarget
   sys.exit(0)

###############################################################################

# Borrowed from id, this takes a prefix and adds it to each filename and
# then returns them as a list
def buildList(sPrefix, sString):
   sList = Split(sString)
   for i in range(len(sList)):
      sList[i] = sPrefix + '/' + sList[i]
   return sList

# This takes a base path, a subpath include path, a subpath lib path
# and a string of libs, and then adds the appropriate data to our
# internal variables (include path, lib path and list of libs)
def addExternal(basePath, subIncPath, subLibPath, libs): 
   includeDir = Split(basePath + subIncPath)
   libDir = Split(basePath + subLibPath)
   extIncPath.extend(includeDir)
   extLibPath.extend(libDir)
   extLibs.extend(Split(libs))

# Embeds a Visual Studio-style manifest into the given output target
# (only under Windows)
def embedManifest(environment, target, suffix):
   # The suffix indicates the file type (1=.exe, 2=.dll)
   environment.AddPostAction(target, 
                             'mt.exe -nologo -manifest ${TARGET}.manifest \
                             -outputresource:$TARGET;' + str(suffix))

###############################################################################

# Set the initial defines and flags

# Create an exportable environment; this will be used as the basis
# environment for all of the modules we create (though they can add
# additional element to it if necessary)
basisEnv = Environment()

# Set-up defines
defines = Split('');

# Then handle platform-specific issues
if buildTarget == 'win32.32bit':
   # Flags for the VC++ compiler
   # /nologo      = Don't print the compiler banner
   # /MD          = Use multithreaded DLL runtime
   # /O2          = Optimize for speed
   # /EHsc        = Exception handling
   # /W3          = Warning level
   # /Zc:forScope = Use standard C++ scoping rules in for loops
   # /GR          = Enable C++ run-time type information
   # /Gd          = Use __cdecl calling convention
   # /Z7          = Generate debug information
   compileFlags = Split('/nologo /MD /O2 /EHsc /W3 /Zc:forScope /GR /Gd /Z7')

   # Additional flags to disable useless warnings in Windows
   compileFlags += Split('/wd4091 /wd4275 /wd4290 /wd4068')

   # Disable deprecation warnings for "insecure" and "nonstandard" functions
   # in Windows
   defines += Split('_CRT_SECURE_NO_DEPRECATE _CRT_NONSTDC_NO_DEPRECATE')

   # Flags for the VC++ linker
   # /DEBUG             = Generate debugging information
   # /OPT:REF           = Optimize away unreferenced code
   # /OPT:ICF           = Optimize away redundant function packages
   # /INCREMENTAL:NO    = Do not perform incremental linking
   # /SUBSYSTEM:WINDOWS = Create a Windows (not a console) application
   # /MANIFEST          = Generate a manifest file
   linkFlags = Split('/DEBUG /OPT:REF /OPT:ICF /INCREMENTAL:NO \
                      /SUBSYSTEM:WINDOWS /MANIFEST')
elif buildTarget == 'posix.64bit':
   # Flags for gcc (generate debug information and optimize)
   compileFlags = Split('-g -O -Wno-write-strings')

   # Added for FFMPEG to include properly
   defines += Split('__STDC_CONSTANT_MACROS')

   # Flags for linking (these define where the lib and plugin directories
   # are for use at runtime rather than depending on LD_LIBRARY_PATH)
   linkFlags = Split('-z origin -Wl,--rpath='+
                     '\\$$ORIGIN'+'/../lib')
elif buildTarget == 'posix.32bit':
   # Flags for gcc (generate debug information and optimize)
   compileFlags = Split('-g -O -Wno-write-strings')

   # Flags for linking (these define where the lib and plugin directories
   # are for use at runtime rather than depending on LD_LIBRARY_PATH)
   linkFlags = Split('-z origin -Wl,--rpath='+
                     '\\$$ORIGIN'+'/../lib')
else:
   # Unsupported architecture so bail
   print "Unsupported target type ", buildTarget
   sys.exit(0)

# Set the initial paths and libraries used
incPath = Split('')
libPath = Split('')
libs = Split('')

# Add the elements to the environment
basisEnv.Append(CCFLAGS = compileFlags)
basisEnv.Append(CPPDEFINES = defines)
basisEnv.Append(CPPPATH = incPath)
basisEnv.Append(LIBPATH = libPath)
basisEnv.Append(LIBS = libs)
basisEnv.Append(LINKFLAGS = linkFlags)

###############################################################################

# Now, we set-up an environment for PhysX uses (note that this *only* includes
# what is needed for PhysX -- so it would have be merged with another
# environment in actual use)
physxEnv = Environment()

# Initialize the external used library information (include path, lib path
# and libs)
extFlags = basisEnv['CCFLAGS']
extDefines = ['NDEBUG']
extIncPath = []
extLibPath = []
extLibs = []

# Depending on platform, add the external libraries that the plugin requires
# (Windows requires more to be linked in than Linux does)
if buildTarget == 'win32.32bit':
   # Add PhysX
   addExternal(physxPath, '/Include', '/Lib/vc12win64',
               'PhysX3_x64 PhysX3CharacterKinematic_x64 PhysX3Common_x64 PhysX3Cooking_x64 PhysX3Extensions PhysX3Gpu_x64 PhysX3Vehicle');
   addExternal(physxPath, '/Include', '/Bin/vc12win64',
               'PhysX3_x64 PhysX3CharacterKinematic_x64 PhysX3Common_x64 PhysX3Cooking_x64');
   # Add msinttypes headers
   extIncPath.extend(Split(msinttypesPath + '/include'))
elif buildTarget == 'posix.64bit':
   # Determine debug or release mode: default is release or 0 anything else is
   # debug mode
   if int(debug):
      # Add these when compiling for visual debugger of physX
      addExternal(physxPath, '/Include', '/Lib/linux64',
              'LowLevelDEBUG LowLevelClothDEBUG PhysX3ExtensionsDEBUG PhysX3VehicleDEBUG PhysXProfileSDKDEBUG PhysXVisualDebuggerSDKDEBUG PvdRuntimeDEBUG PxTaskDEBUG SceneQueryDEBUG SimulationControllerDEBUG');
      addExternal(physxPath, '/Include', '/Bin/linux64',
              'PhysX3CharacterKinematicDEBUG_x64 PhysX3CommonDEBUG_x64 PhysX3CookingDEBUG_x64 PhysX3DEBUG_x64 PhysX3GpuDEBUG_x64');      
   else:
     # Add these when compiling for release
      addExternal(physxPath, '/Include', '/Lib/linux64',
              'LowLevel LowLevelCloth PhysX3Extensions PhysX3Vehicle PhysXProfileSDK PhysXVisualDebuggerSDK PvdRuntime PxTask SceneQuery SimulationController');
      addExternal(physxPath, '/Include', '/Bin/linux64',
              'PhysX3CharacterKinematic_x64 PhysX3Common_x64 PhysX3Cooking_x64 PhysX3_x64 PhysX3Gpu_x64'); 
elif buildTarget == 'posix.32bit':
   addExternal(physxPath, '/Include', '/Lib/linux32',
               'PhysX3Extensions PhysX3Vehicle PhysXProfileSDK PhysXVisualDebuggerSDK PxTask');
   addExternal(physxPath, '/Include', '/Bin/linux32',
               'PhysX3CharacterKinematic_x32 PhysX3Common_x32 PhysX3Cooking_x32 -lPhysX3_x32');
else:
   # Unsupported architecture so bail
   print "UnsupportedC target type ", buildTarget
   sys.exit(0)


# Add ATLAS
atlasIncPath = buildList(atlasPath, 'communication container foundation math os util xml')
extIncPath.extend(atlasIncPath)
extLibPath.extend(Split(atlasPath))
extLibs.extend(Split('atlas'))

extDefines.extend(Split('ATLAS_SYM=IMPORT'))

# Add the elements to the PhysX environment
physxEnv.Append(CCFLAGS = extFlags)
physxEnv.Append(CPPDEFINES = extDefines)
physxEnv.Append(CPPPATH = extIncPath)
physxEnv.Append(LIBPATH = extLibPath)
physxEnv.Append(LIBS = extLibs)



###############################################################################

# Create the libraries
libTuple = SConscript(['libsrc/SConscript'], 'basisEnv physxEnv buildList')
libObjs = libTuple[0]
libEnv = libTuple[1]
libLib = libEnv.SharedLibrary('lib/PhysX', libObjs)
if buildTarget == 'win32.32bit':
   embedManifest(libEnv, libLib, 2)


# Finally, create the main application
#appTuple = SConscript(['src/SConscript'], 
#                      'basisEnv buildList')
#appObjs = appTuple[0]
#appEnv = appTuple[1]
#mainApp = appEnv.Program('bin/main.exe', appObjs)
#if buildTarget == 'win32.32bit':
#   embedManifest(appEnv, mainApp, 1)

