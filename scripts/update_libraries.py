import shutil

LIB_HEADER_PATH = r"../libraries/include"
LIB_SOURCE_PATH = r"../libraries/src"

projects = [
    "APPS",
    "BMS",
    "LVPDB",
    "STEERINGWHEEL",
    "CANalyze",
    ]



for project in projects:
    print("Updating {0}...".format(project))
    header_target_path = r"../{0}/Core/Inc".format(project)
    shutil.copytree(LIB_HEADER_PATH, header_target_path, dirs_exist_ok=True)
    
    source_target_path = r"../{0}/Core/Src".format(project)
    shutil.copytree(LIB_SOURCE_PATH, source_target_path, dirs_exist_ok=True)


input("Press ENTER to exit")
