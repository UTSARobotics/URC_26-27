# ros3 build and installation guide (with fixes)

This guide includes the build issues encountered when building from sibling source folders without installing Mirage and Quicksand system-wide.

## 1. Install build tools

On Debian or Ubuntu:

```sh
sudo apt update
sudo apt install git build-essential clang llvm lld ninja-build scdoc
```

`ninja-build` is needed by the `compile_commands.json` target used during `make check`. `lld` is needed because the Makefile passes `-fuse-ld=lld`.

## 2. Clone the three repositories as sibling folders

For example:

```text
~/mirage
~/quicksand
~/ros3
```

Clone and build Mirage:

```sh
cd ~
git clone https://git.sr.ht/~alecgraves/mirage
cd mirage
make
make check
make docs
```

Then Quicksand:

```sh
cd ~
git clone https://git.sr.ht/~alecgraves/quicksand
cd quicksand
make
make check
make docs
```

Then ros3:

```sh
cd ~
git clone https://git.sr.ht/~alecgraves/ros3
cd ros3
make
make check
make docs
```

## 3. If you are building without installing dependencies

The ros3 Makefile defaults to `../mirage` and `../quicksand`. If the repositories are siblings, those defaults should work. In `~/ros3/Makefile`, make sure the dependency settings point to the local build outputs:

```make
DEP_SHARED_LIBS := \
	-L$(abspath $(MIRAGE_DIR)/build) \
	-L$(abspath $(QUICKSAND_DIR)/build) \
	-L$(PREFIX)/lib \
	-Wl,-rpath,$(abspath $(MIRAGE_DIR)/build) \
	-Wl,-rpath,$(abspath $(QUICKSAND_DIR)/build) \
	-Wl,-rpath,$(PREFIX)/lib \
	-lmirage -lquicksand

DEP_STATIC_LIBS := $(abspath $(MIRAGE_DIR)/build/libmirage.a) \
                   $(abspath $(QUICKSAND_DIR)/build/libquicksand.a)
```

These settings address two separate linker errors: `-lmirage`/`-lquicksand` not being found, and the executable link rules looking for static libraries under `/usr/local/lib`.

For `rosetopic`, ensure its link command includes `-lm` at the end, because it uses `sqrt`. The Makefile should define:

```make
ROSETOPIC_LIBS := -lm
```

and the `rosetopic` recipe should end with `$(ROSETOPIC_LIBS)`.

## 4. Build and test the Python package

The Python extension needs to find the dependency headers. In `~/ros3/lang/python/setup.py`, its `include_dirs` should include the local Mirage and Quicksand headers:

```python
include_dirs=[
    os.path.join(ROOT, "include"),
    os.path.abspath(os.path.join(ROOT, "..", "mirage", "include")),
    os.path.abspath(os.path.join(ROOT, "..", "quicksand", "include")),
    "/usr/local/include",
],
```

Then run:

```sh
cd ~/ros3
make python
```

If Python tests report that `libmirage.so`, `libquicksand.so`, or `libros3.so` cannot be found, run them with all three build directories in the runtime library path:

```sh
LD_LIBRARY_PATH="$HOME/ros3/build:$HOME/mirage/build:$HOME/quicksand/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" make check-python
```

To make `make check-python` set this automatically, change its Makefile rule to:

```make
check-python: $(BUILD_DIR)/libros3.so
	LD_LIBRARY_PATH="$(abspath $(BUILD_DIR)):$(abspath $(MIRAGE_DIR)/build):$(abspath $(QUICKSAND_DIR)/build):$$LD_LIBRARY_PATH" python3 test/python/test_pubsub.py
```

Keep the indentation before the command as a **tab**.

## 5. Docs and common messages

`make docs` saying `Nothing to be done for 'docs'` means Make considers the documentation targets current. To force regeneration, run:

```sh
make -B docs
```

Warnings about `pytest_repeat` metadata or deprecated `setup.py` commands did not stop the Python package from installing in your run. The important success message was `Successfully installed ros3-1.0.0`.
