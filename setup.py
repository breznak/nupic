import glob
import numpy
import os
import re
import setuptools
import shutil
import sys
import tarfile
import urllib2

from setuptools import setup, find_packages, Extension

"""
This file builds and installs the NuPIC binaries.
"""

NUPIC_CORE_BUCKET = (
  "https://s3-us-west-2.amazonaws.com/artifacts.numenta.org/numenta/nupic.core"
)
REPO_DIR = os.path.dirname(os.path.realpath(__file__))
DARWIN_PLATFORM = "darwin"
LINUX_PLATFORM = "linux"
UNIX_PLATFORMS = [LINUX_PLATFORM, DARWIN_PLATFORM]
WINDOWS_PLATFORMS = ["windows"]



def downloadFile(url, destFile, silent=False):
  """
  Download a file to the specified location
  """

  if not silent:
    print "Downloading from\n\t%s\nto\t%s.\n" % (url, destFile)

  destDir = os.path.dirname(destFile)
  if not os.path.exists(destDir):
    os.makedirs(destDir)

  try:
    response = urllib2.urlopen(url)
  except urllib2.URLError:
    return False

  with open(destFile, "wb") as fileObj:
    totalSize = response.info().getheader("Content-Length").strip()
    totalSize = int(totalSize)
    bytesSoFar = 0

    # Download chunks writing them to target file
    chunkSize = 8192
    oldPercent = 0
    while True:
      chunk = response.read(chunkSize)
      bytesSoFar += len(chunk)

      if not chunk:
        break

      fileObj.write(chunk)

      # Show progress
      if not silent:
        percent = (float(bytesSoFar) / totalSize) * 100
        percent = int(percent)
        if percent != oldPercent and percent % 5 == 0:
          print ("Downloaded %i of %i bytes (%i%%)."
                 % (bytesSoFar, totalSize, int(percent)))
          oldPercent = percent

  return True



def unpackFile(package, dirToUnpack, destDir, silent=False):
  """
  Unpack package file to the specified directory
  """

  if not silent:
    print "Unpacking %s into %s..." % (package, destDir)

  with tarfile.open(package, "r:gz") as tarFileObj:
    tarFileObj.extractall(destDir)

  # Copy subdirectories to a level up
  subDirs = os.listdir(destDir + "/" + dirToUnpack)
  for subDir in subDirs:
    shutil.rmtree(destDir + "/" + subDir, True)
    shutil.move(destDir + "/" + dirToUnpack + "/" + subDir,
                destDir + "/" + subDir)

  shutil.rmtree(destDir + "/" + dirToUnpack, True)



def printOptions(optionsDesc):
  """
  Print command line options.
  """

  print "Options:\n"

  for option in optionsDesc:
    optionUsage = "--" + option[0]
    if option[1] != "":
      optionUsage += "=[" + option[1] + "]"

    optionDesc = option[2]
    print "    " + optionUsage.ljust(30) + " = " + optionDesc



def getCommandLineOptions():

  # optionDesc = [name, value, description]
  optionsDesc = []
  optionsDesc.append(
    ["nupic-core-dir",
     "dir",
     "(optional) Absolute path to nupic.core binary release directory"]
  )
  optionsDesc.append(
    ["skip-compare-versions",
     "",
     "(optional) Skip nupic.core version comparison"]
  )

  # Read command line options looking for extra options
  # For example, an user could type:
  #   python setup.py install --nupic-core-dir="path/to/release"
  # which will set the nupic.core release dir
  optionsValues = dict()
  for arg in sys.argv[:]:
    optionFound = False
    for option in optionsDesc:
      name = option[0]
      if "--" + name in arg:
        value = None
        hasValue = (option[1] != "")
        if hasValue:
          value = arg.partition("=")[2]

        optionsValues[name] = value
        sys.argv.remove(arg)
        optionFound = True
        break

    if not optionFound:
      if ("--help-nupic" in arg):
        printOptions(optionsDesc)
        sys.exit()

  return optionsValues



def getPlatformInfo():
  """
  Identify platform
  """

  if "linux" in sys.platform:
    platform = "linux"
  elif "darwin" in sys.platform:
    platform = "darwin"
  elif "windows" in sys.platform:
    platform = "windows"
  else:
    raise Exception("Platform '%s' is unsupported!" % sys.platform)

  if sys.maxsize > 2**32:
    bitness = "64"
  else:
    bitness = "32"

  return platform, bitness



def getVersion():
  """
  Get version from local file.
  """
  with open("VERSION", "r") as versionFile:
    return versionFile.read().strip()


def parse_file(requirementFile):
  try:
    return [
      line.strip()
      for line in open(requirementFile).readlines()
      if not line.startswith("#")
    ]
  except IOError:
    return []


def findRequirements(nupicCoreReleaseDir):
  """
  Read the requirements.txt file and parse into requirements for setup's
  install_requirements option.
  """
  requirementsPath = os.path.join(REPO_DIR, "external/common/requirements.txt")
  coreRequirementsPath = os.path.join(nupicCoreReleaseDir, "requirements.txt")
  
  dependencies = []
  # use develop nupiccore bindings. not PYPI
  eggFiles = glob.glob(os.path.join(nupicCoreReleaseDir, "*.egg"))
  for egg in eggFiles:
    from wheel import egg2wheel
    egg2wheel.egg2wheel(egg, nupicCoreReleaseDir)
    dependencies.append(egg)

  wheelFiles = glob.glob(os.path.join(nupicCoreReleaseDir, "*.whl"))
  for wheel in wheelFiles:
    dependencies.append(wheel)

  requirements = parse_file(requirementsPath)

  requirements += parse_file(coreRequirementsPath)
   
  return requirements, dependencies



def extractNupicCoreTarget():
  # First, get the nupic.core SHA and remote location from local config.
  nupicConfig = {}
  if os.path.exists(REPO_DIR + "/.nupic_config"):
    execfile(
      os.path.join(REPO_DIR, ".nupic_config"), {}, nupicConfig
    )
  elif os.path.exists(os.environ["HOME"] + "/.nupic_config"):
    execfile(
      os.path.join(os.environ["HOME"], ".nupic_config"), {}, nupicConfig
    )
  else:
    execfile(
      os.path.join(REPO_DIR, ".nupic_modules"), {}, nupicConfig
    )
  return nupicConfig["NUPIC_CORE_COMMITISH"]



def getDefaultNupicCoreDirectories():
  # Default nupic.core location is relative to the NuPIC checkout.
  return (
    REPO_DIR + "/extensions/core/build/release",
    REPO_DIR + "/extensions/core"
  )



def getCommandLineOption(name, options):
  if name is None or options is None:
    return False
  if name in options:
    return options[name]



def prepareNupicCore(options, platform, bitness):

  nupicCoreReleaseDir = getCommandLineOption("nupic-core-dir", options)
  if nupicCoreReleaseDir is not None:
    nupicCoreReleaseDir = os.path.expanduser(nupicCoreReleaseDir)
  nupicCoreSourceDir = None
  fetchNupicCore = True

  if nupicCoreReleaseDir:
    # User specified that they have their own nupic.core
    fetchNupicCore = False
  else:
    nupicCoreReleaseDir, nupicCoreSourceDir = getDefaultNupicCoreDirectories()

  nupicCoreCommitish = extractNupicCoreTarget()

  if fetchNupicCore:
    # User has not specified 'nupic.core' location, so we'll download the
    # binaries.

    nupicCoreRemoteUrl = (NUPIC_CORE_BUCKET + "/nupic_core-"
                          + nupicCoreCommitish + "-" + platform + bitness + ".tar.gz")
    nupicCoreLocalPackage = (nupicCoreSourceDir + "/nupic_core-"
                             + nupicCoreCommitish + "-" + platform + bitness + ".tar.gz")
    if getPlatformInfo()[0] == "darwin":
      nupicCoreLocalDirToUnpack = "/Users/travis/build/numenta/nupic.core/bindings/py/dist"
    elif getPlatformInfo()[0] == "linux":
      nupicCoreLocalDirToUnpack = "/home/travis/build/numenta/nupic.core/bindings/py/dist"

    if os.path.exists(nupicCoreLocalPackage):
      print ("Target nupic.core package already exists at "
             + nupicCoreLocalPackage + ".")
      unpackFile(
        nupicCoreLocalPackage, nupicCoreLocalDirToUnpack, nupicCoreReleaseDir
      )
    else:
      print "Attempting to fetch nupic.core binaries..."
      downloadSuccess = downloadFile(
        nupicCoreRemoteUrl, nupicCoreLocalPackage
      )

      # TODO: Give user a way to clean up all the downloaded binaries. It can
      # be manually done with `rm -rf $NUPIC_CORE/extensions/core` but would
      # be cleaner with something like `python setup.py clean`.

      if not downloadSuccess:
        raise Exception("Failed to download nupic.core tarball from %s}! "
                        "Ensure you have an internet connection and that the "
                        "remote tarball exists." % nupicCoreRemoteUrl)
      else:
        print "Download successful."
        unpackFile(nupicCoreLocalPackage,
                        nupicCoreLocalDirToUnpack,
                        nupicCoreReleaseDir)

  else:
    print "Using nupic.core binaries at " + nupicCoreReleaseDir

  if getCommandLineOption("skip-compare-versions", options):
    skipCompareVersions = True
  else:
    skipCompareVersions = not fetchNupicCore

  if not skipCompareVersions:
    # Compare expected version of nupic.core against installed version
    with open(nupicCoreReleaseDir + "/include/nupic/Version.hpp",
              "r") as fileObj:
      content = fileObj.read()

    nupicCoreVersionFound = re.search(
      "#define NUPIC_CORE_VERSION \"([a-z0-9]+)\"", content
    ).group(1)

    if nupicCoreCommitish != nupicCoreVersionFound:
      raise Exception(
        "Fatal Error: Unexpected version of nupic.core! "
        "Expected %s, but detected %s."
        % (nupicCoreCommitish, nupicCoreVersionFound)
      )

  return nupicCoreReleaseDir



def copyProtoFiles(nupicCoreReleaseDir):
  # Copy proto files located at nupic.core dir into nupic dir
  print "Copying capnp files from nupic core"
  protoSourceDir = glob.glob(os.path.join(nupicCoreReleaseDir, "include/nupic/proto/"))[0]
  protoTargetDir = REPO_DIR + "/nupic/bindings/proto"
  if not os.path.exists(protoTargetDir):
    os.makedirs(protoTargetDir)
  for fileName in glob.glob(protoSourceDir + "/*.capnp"):
    shutil.copy(fileName, protoTargetDir)



def postProcess():
  # Copy binaries located at nupic.core dir into source dir
  print ("Copying binaries from " + nupicCoreReleaseDir + "/bin" + " to "
         + REPO_DIR + "/bin...")
  if not os.path.exists(REPO_DIR + "/bin"):
    os.makedirs(REPO_DIR + "/bin")

  for binFile in glob.glob(nupicCoreReleaseDir + "/bin/*"):
    shutil.copy(binFile, REPO_DIR + "/bin")

if __name__ == "__main__":
  print "setuptools version: {}".format(setuptools.__version__)
  print "numpy version: {}".format(numpy.__version__)

  options = getCommandLineOptions()
  platform, bitness = getPlatformInfo()

  # Build and setup NuPIC
  cwd = os.getcwd()
  os.chdir(REPO_DIR)

  try:
    haveBuild = False
    buildCommands = ["build", "install", "develop", "bdist", "bdist_wheel"]
    for arg in sys.argv[:]:
      if arg in buildCommands:
        haveBuild = True

    nupicCoreReleaseDir = prepareNupicCore(options, platform, bitness)
    print "nupic core release directory: {}".format(nupicCoreReleaseDir)

    copyProtoFiles(nupicCoreReleaseDir)

    requirements, dependencies = findRequirements(nupicCoreReleaseDir)
    setup(
      name="nupic",
      version=getVersion(),
      install_requires=requirements,
      dependency_links=dependencies,
      packages=find_packages(),
      namespace_packages = ["nupic", "nupic.bindings"],
      package_data={
        "nupic.support": ["nupic-default.xml",
                          "nupic-logging.conf"],
        "nupic": ["README.md", "LICENSE.txt"],
        "nupic.data": ["*.json"],
        "nupic.frameworks.opf.exp_generator": ["*.json", "*.tpl"],
        "nupic.frameworks.opf.jsonschema": ["*.json"],
        "nupic.swarming.exp_generator": ["*.json", "*.tpl"],
        "nupic.swarming.jsonschema": ["*.json"],
        "nupic.datafiles": ["*.csv", "*.txt"],
        "nupic.encoders": ["*.capnp"],
        "nupic.bindings.proto": ["*.capnp"],
      },
      include_package_data=True,
      zip_safe=False,
      description="Numenta Platform for Intelligent Computing",
      author="Numenta",
      author_email="help@numenta.org",
      url="https://github.com/numenta/nupic",
      classifiers=[
        "Programming Language :: Python",
        "Programming Language :: Python :: 2",
        "License :: OSI Approved :: GNU General Public License (GPL)",
        "Operating System :: MacOS :: MacOS X",
        "Operating System :: POSIX :: Linux",
        # It has to be "5 - Production/Stable" or else pypi rejects it!
        "Development Status :: 5 - Production/Stable",
        "Environment :: Console",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Artificial Intelligence"
      ],
      long_description = """\
    Numenta Platform for Intelligent Computing: a machine intelligence platform that implements the HTM learning algorithms. HTM is a detailed computational theory of the neocortex. At the core of HTM are time-based continuous learning algorithms that store and recall spatial and temporal patterns. NuPIC is suited to a variety of problems, particularly anomaly detection and prediction of streaming data sources.

    For more information, see http://numenta.org or the NuPIC wiki at https://github.com/numenta/nupic/wiki.
    """)

    if haveBuild:
      postProcess()
  finally:
    os.chdir(cwd)
