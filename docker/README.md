## Registry and publishing

The CONCERT image is published to
`ghcr.io/advrhumanoids/concert_description`. The XBot2 base image continues to
come from Docker Hub; existing CONCERT images on Docker Hub are not removed.
The GHCR image becomes available after the first successful publication.

The workflow in `.github/workflows/docker.yml` builds pull requests targeting
`ros2` without logging in or publishing images. On push, it publishes:

- `latest` for the `ros2` branch;
- `ros1-legacy` for the `master` branch, preserving the existing tagging rule;
- the Git tag for release tags matching `v[0-9]+.[0-9]+.[0-9]+*`;
- an additional `sha-<full-commit-sha>` tag for each published build.

Authentication uses the workflow's `GITHUB_TOKEN` with `packages: write`;
Docker Hub credentials and a separate personal token are not required for
publishing. In GitHub's package settings, grant the workflow repository Write
access under **Manage Actions access**, or use inherited repository access.
The image's `org.opencontainers.image.source` label identifies the source
repository. The package's visibility is managed separately; public images can
be pulled anonymously, while private images require an authenticated account
with package access. See the
[GitHub Container Registry documentation](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry).

The build and run helpers default to GHCR. To use a different image repository,
set `IMAGE` (without a tag) when invoking either helper, for example:

```bash
IMAGE=concert_description-dev ./docker/build-docker.bash
IMAGE=concert_description-dev ./docker/run-docker.bash
```

## Build and run

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
docker run --rm ghcr.io/advrhumanoids/concert_description:latest \
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
