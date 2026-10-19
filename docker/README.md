# CONCERT on Noble / ROS 2 Jazzy

The image extends `hhcmhub/xbot2-noble-dev:latest`. It inherits ROS Jazzy,
XBot2, and `/home/user/env`. The base's `ros-jazzy-desktop-full` already
provides the required Gazebo integration through `ros-jazzy-simulation`.
This image adds CONCERT's source dependencies and the ROS 2 packages from
this checkout.

From the repository root, initialize the resources submodule and build:

```bash
git submodule update --init --recursive
./docker/build-docker.bash
```

To use your own base, first build the upstream Noble Dockerfile from a checkout
of https://github.com/ADVRHumanoids/xbot2_docker:

```bash
docker build --build-arg USER_ID="$(id -u)" -t xbot2-noble:local ./noble
```

Then, from this repository:

```bash
./docker/build-docker.bash --build-arg BASE_IMAGE=xbot2-noble:local
```

The derived image inherits the base user's UID. Build the base with the host
UID when using writable bind mounts. For release builds, pass a tested base
tag/digest and `--build-arg RECIPES_REF=<tested-ros2-recipes-revision>`.
`RECIPES_REPO` and `BUILD_JOBS` are also configurable build arguments. Source
dependencies use the revisions selected by the recipes; pin those recipes'
source references as well when making a reproducible release. Local changes
in sibling repositories such as Modular are not included by this build.

Dependencies are maintained in the Forest recipes, not in the Dockerfile.
The Dockerfile explicitly sources `/opt/xbot/setup.sh` before `forest grow`
so Forest can discover the base image's installed XBot2 dependencies. It also
checks that `xbot2-core` is available before starting the build.
`forest grow concert_all` builds the dependency tree and this checkout together;
changing CONCERT source invalidates that Docker build layer. Add or remove
CONCERT dependencies in `multidof_recipes/recipes/concert_description.yaml`.
The recipes revision used for the image must include the camera description
and GUI server dependencies. Publish recipe changes to `RECIPES_REPO` and
select them with `RECIPES_REF`; local edits in the sibling recipes repository
are not copied into the image.

The build context is this repository, and the Dockerfile-specific ignore file
includes only the selected ROS 2 packages and Docker helpers. Local edits to
these packages, including the initialized `concert_resources` submodule, are
included. The legacy ROS 1 packages and the separate Isaac environment are
excluded.

The entrypoint sources ROS, `/opt/xbot/setup.sh` when present, the Python
environment, and the Forest workspace. This also applies to commands passed
directly to `docker run`. For example:

```bash
docker run --rm edoardoromiti/concert_description:latest \
    ros2 pkg prefix concert_gazebo
```

Inside an interactive container with GPU/display access, launch with:

```bash
ros2 launch concert_gazebo modular.launch.py
```

The `ros_gz_sim` launcher reads the resource path exported by
`concert_resources/package.xml` to resolve `model://concert_resources/...`
meshes. It also includes `LD_LIBRARY_PATH` in Gazebo's system plugin search
path, using the libraries exposed by the XBot2 and Forest setup scripts.

The existing `run-docker.bash` supplies NVIDIA/X11 access and starts a terminal.
It also sets `__NV_PRIME_RENDER_OFFLOAD=1` and
`__GLX_VENDOR_LIBRARY_NAME=nvidia` so OpenGL applications request NVIDIA on
hybrid Intel/NVIDIA systems. These are runtime settings; changing the script
does not require rebuilding the image. To test in an already running container:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia rviz2
```

If graphics still fail, check `nvidia-smi` inside the container. If `glxinfo`
is available, `glxinfo -B` should report an NVIDIA OpenGL renderer; `nvidia-smi`
alone verifies GPU visibility, not OpenGL renderer selection. See NVIDIA's
[PRIME render offload documentation](https://download.nvidia.com/XFree86/Linux-x86_64/570.169/README/primerenderoffload.html).

Communicating with external ROS nodes may require additional network/DDS
configuration. The GUI launcher configuration must also use container-valid
paths if it contains absolute host workspace paths.
