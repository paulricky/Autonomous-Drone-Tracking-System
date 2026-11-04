"""Compatibility entry point for the native authenticated operator gateway."""
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix


def main(args=None):
    executable = Path(get_package_prefix("robotics_core")) / "lib/robotics_core/operator_receiver_node"
    # Replace this process so signals and the native watchdog lifecycle are preserved.
    os.execv(str(executable), [str(executable), *(sys.argv[1:] if args is None else args)])


if __name__ == "__main__":
    main()
