import os
import subprocess
import sys

Import("env")


if sys.platform.startswith("win"):
    os.environ.setdefault("IDF_COMPONENT_CACHE_PATH", r"C:\icm")
else:
    os.environ.setdefault("IDF_COMPONENT_CACHE_PATH", os.path.expanduser("~/.cache/idf_component_manager"))


def _generate_text_data_stub(data_path, output_name=None, fallback_path=None):
    project_dir = env.subst("$PROJECT_DIR")
    build_dir = env.subst("$BUILD_DIR")
    source_path = os.path.join(project_dir, data_path)
    if not os.path.exists(source_path) and fallback_path:
        source_path = os.path.join(project_dir, fallback_path)
    if not os.path.exists(source_path):
        return

    output_base = output_name or os.path.basename(data_path)
    asm_path = os.path.join(build_dir, f"{output_base}.S")
    if os.path.exists(asm_path):
        return

    os.makedirs(build_dir, exist_ok=True)
    cmake_name = "cmake.exe" if sys.platform.startswith("win") else "cmake"
    cmake = os.path.join(env.PioPlatform().get_package_dir("tool-cmake"), "bin", cmake_name)
    script = os.path.join(
        env.PioPlatform().get_package_dir("framework-espidf"), "tools", "cmake", "scripts", "data_file_embed_asm.cmake"
    )
    subprocess.run(
        [
            cmake,
            f"-DDATA_FILE={source_path}",
            f"-DSOURCE_FILE={asm_path}",
            "-DFILE_TYPE=TEXT",
            "-P",
            script,
        ],
        check=True,
    )


_generate_text_data_stub(os.path.join("main", "data", "module_drivers.json"))
_generate_text_data_stub(
    os.path.join("managed_components", "espressif__esp_insights", "server_certs", "https_server.crt"),
    output_name="https_server.crt",
    fallback_path=os.path.join("main", "data", "module_drivers.json"),
)
