import os

import shutil

dirname = os.path.dirname(os.path.abspath(__file__))

shutil.copy(os.path.join(dirname, r".pioenvs\nucleo_f401re\firmware.bin"), r"D:\\")