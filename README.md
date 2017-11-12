# taco

Main repository for the autonomous driving project.

## Project Structure

- `server` - C++ server, interacting with ADTF
- `client` - Java client, using the same [base classes](https://github.com/hsoautonomy/base) as magma
- `vision` - Python object detection, using TensorFlow
- `config` - JSON description of the supported car models

## Tools

We use `clang-format` for code formatting. Instructions can be found [here](https://github.com/hsoautonomy/formatting).

The recommended IDEs for the different projects are:

- `server` - CLion
- `client` - IntelliJ IDEA (or Eclipse)
- `vision` - PyCharm (or IntelliJ with the Python plugin)

It's also strongly recommended to use a dedicated Git client:

- Windows / Mac: SourceTree
- Linux: GitKraken

In SourceTree, the following default settings in `Tools -> Options -> Git` should be changed:

- Enable `Use rebase instead of merge by default for tracked branches`
- Enable `Peform submodule actions recursively`
