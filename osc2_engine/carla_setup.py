"""
CARLA Version Setup.

Configures sys.path to load the correct CARLA Python API based on the selected version.
Must be called BEFORE importing carla or any engine modules.
"""

import sys

CARLA_VERSIONS = {
    "0.10.0": {
        "python_api": "/home/hnh21/iotav/CARLA_0.10/CARLA_0.10.0/PythonAPI/carla",
        "carla_module": None,  # System-installed (pip)
        "launch_script": "/home/hnh21/iotav/CARLA_0.10/CARLA_0.10.0/CarlaUnreal.sh --ros2",
    },
    "0.9.16": {
        "python_api": "/home/hnh21/iotav/CARLA_0.10/CARLA_0.9.16/PythonAPI/carla",
        "carla_module": "/home/hnh21/iotav/CARLA_0.10/CARLA_0.9.16/PythonAPI/carla/dist/extracted",
        "launch_script": "/home/hnh21/iotav/CARLA_0.10/CARLA_0.9.16/CarlaUE4.sh --ros2",
    },
}


def setup_carla(version):
    """
    Configure sys.path for the specified CARLA version.

    Must be called before any `import carla` statement.
    """
    if version not in CARLA_VERSIONS:
        raise ValueError(f"Unknown CARLA version: {version}. Available: {list(CARLA_VERSIONS.keys())}")

    config = CARLA_VERSIONS[version]

    # For 0.9.16: insert extracted wheel path BEFORE site-packages
    # so it takes priority over the system-installed 0.10.0
    if config["carla_module"]:
        sys.path.insert(0, config["carla_module"])

    # Add PythonAPI/carla for agents module
    if config["python_api"] not in sys.path:
        sys.path.insert(0 if config["carla_module"] else len(sys.path), config["python_api"])

    # Verify import works
    import carla
    return config


def prompt_version():
    """Interactively prompt the user to select a CARLA version."""
    print("Available CARLA versions:")
    versions = list(CARLA_VERSIONS.keys())
    for i, v in enumerate(versions, 1):
        info = CARLA_VERSIONS[v]
        engine = "UE5" if "0.10" in v else "UE4"
        print(f"  [{i}] CARLA {v} ({engine}) — {info['launch_script'].split('/')[-1].split(' ')[0]}")

    while True:
        try:
            choice = input(f"Select version [1-{len(versions)}]: ").strip()
            idx = int(choice) - 1
            if 0 <= idx < len(versions):
                return versions[idx]
        except (ValueError, EOFError):
            pass
        print(f"  Please enter a number between 1 and {len(versions)}")
