<div align="center">
    <img title="Numenta Logo" src="http://numenta.org/images/250x250numentaicon.gif"/>
</div>

# Numenta Platform for Intelligent Computing (NuPIC)

[![Build Status](https://travis-ci.org/numenta/nupic.png?branch=master)](https://travis-ci.org/numenta/nupic)

NuPIC is a library that provides the building blocks for online prediction systems.  The library contains the Cortical Learning Algorithm (CLA), but also the Online Prediction Framework (OPF) that allows clients to build prediction systems out of encoders, models, and metrics.

For more information, see [numenta.org](http://numenta.org) or the [Github wiki](https://github.com/numenta/nupic/wiki).

Issue tracker at [issues.numenta.org](https://issues.numenta.org/browse/NPC).

## OPF Basics

For more detailed documentation, see the [OPF wiki page](https://github.com/numenta/nupic/wiki/Online-Prediction-Framework).

__Encoders__ turn raw values into sparse distributed representations (SDRs).  A good encoder will capture the semantics of the data type in the SDR using overlapping bits for semantically similar values.

__Models__ take sequences of SDRs and make predictions.  The CLA is implemented as an OPF model.

__Metrics__ take input values and predictions and output scalar representations of the quality of the predictions.  Different metrics are suitable for different problems.

__Clients__ take input data and feed it through encoders, models, and metrics and store or report the resulting predictions or metric results.

## Installation

For all installation options, see the [Getting Started](https://github.com/numenta/nupic/wiki/Getting-Started) wiki page.

Currently supported platforms:
 * Linux (32/64bit)
 * Mac OSX
 * Raspberry Pi (ARMv6)
 * Chromebook (Ubuntu ARM, Crouton) (ARMv7)
 * [VM images](https://github.com/numenta/nupic/wiki/Running-Nupic-in-a-Virtual-Machine)

Dependencies:
 * Python (2.6-2.7) (with development headers)
 * GCC (4.6-4.8), or Clang
 * Make

The dependencies are included in platform-specific repositories for convenience:

* [nupic-linux64](https://github.com/numenta/nupic-linux64) for 64-bit Linux systems
* [nupic-darwin64](https://github.com/numenta/nupic-darwin64) for 64-bit OS X systems

Add the following to your .bashrc file. Change the paths as needed.

    # Installation path
    export NTA=$HOME/nta/eng
    # Target source/repo path. Defaults to $PWD
    export NUPIC=/path/to/repo
    # Convenience variable for temporary build files
    export BUILDDIR=/tmp/ntabuild
    # Number of jobs to run in parallel (optional)
    export MK_JOBS=3

    # Set up the rest of the necessary env variables. Must be done after
    # setting $NTA.
    source $NUPIC/env.sh

If you plan on making changes to NuPIC, add the following to your .bashrc file.

    # Developer mode: make build use symbolic links from source for Python files instead of copying files
    export NTAX_DEVELOPER_BUILD=1

Complete set of python requirements are documented in [requirements.txt](/external/common/requirements.txt),
compatible with [pip](http://www.pip-installer.org/en/latest/cookbook.html#requirements-files):

    pip install -r external/common/requirements.txt

_Note_: If using pip 1.5 or later:

    pip install --allow-all-external --allow-unverified PIL --allow-unverified psutil -r external/common/requirements.txt

_Note_: If you get a "permission denied" error when using pip, add the --user flag.

Build and install NuPIC:

    $NUPIC/build.sh

NuPIC should now be installed in $NTA! If the build failed, check to make sure that $NUPIC is set, and the value is the proper path to the local NuPIC repo.

## Try it out!

### Tests

Run the C++ tests:

    $NTA/bin/htmtest
    $NTA/bin/testeverything

Run the Python unit tests:

    cd $NTA
    ./bin/run_tests.sh

### Examples

You can run the examples using the OpfRunExperiment OPF client:

    python $NUPIC/examples/opf/bin/OpfRunExperiment.py $NUPIC/examples/opf/experiments/multistep/hotgym/

There are also some sample OPF clients. You can modify these to run your own
data sets. One example is the hotgym prediction client:

    python $NUPIC/examples/opf/clients/hotgym/hotgym.py

Also check out other uses of the CLA on the [Getting Started](https://github.com/numenta/nupic/wiki/Getting-Started#next-steps) wiki page.
