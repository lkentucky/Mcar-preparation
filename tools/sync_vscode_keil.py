"""Generate VS Code firmware IntelliSense from the actual uVision target.

Run again after changing branches/project options. --check runs the same Arm
compiler in syntax-only mode; it neither flashes the board nor hides errors.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import shlex
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "project/mdk/rt1064.uvprojx"


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def workspace_path(path):
    try:
        return "${workspaceFolder}/" + path.relative_to(ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def enabled(element):
    return element is None or element.findtext(".//CommonProperty/IncludeInBuild") != "0"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", default="nor_sdram_zf_dtcm")
    parser.add_argument("--compiler", default="C:/Keil_v5/ARM/ARMCLANG/bin/armclang.exe")
    parser.add_argument("--check", action="store_true")
    options = parser.parse_args()
    compiler = Path(options.compiler).resolve()
    if not compiler.is_file():
        raise SystemExit(f"Arm compiler not found: {compiler}")
    targets = ET.parse(PROJECT).findall("./Targets/Target")
    target = next((t for t in targets if t.findtext("TargetName") == options.target), None)
    if target is None:
        raise SystemExit(f"Keil target not found: {options.target}")
    controls = target.find("./TargetOption/TargetArmAds/Cads/VariousControls")
    if controls is None:
        raise SystemExit("Missing Keil C compiler options")
    # This project has no file/group-specific C compiler overrides. Never silently
    # ignore them if a future branch adds such options.
    if target.findall(".//FileArmAds/Cads") or target.findall(".//GroupArmAds/Cads"):
        raise SystemExit("Per-file/group C options found; update the exporter before generating")
    includes = [(PROJECT.parent / p.strip().replace("\\", "/")).resolve()
                for p in controls.findtext("IncludePath", "").split(";") if p.strip()]
    missing = [p for p in includes if not p.is_dir()]
    if missing:
        raise SystemExit("Missing include directories: " + ", ".join(map(str, missing)))
    defines = [re.sub(r"\s*=\s*", "=", p.strip())
               for p in controls.findtext("Define", "").split(",") if p.strip()]
    undefines = [p.strip() for p in controls.findtext("Undefine", "").split(",") if p.strip()]
    # Matches this target's Cortex-M7 + double-precision FPU. C99 is confirmed
    # by DW_AT_language in the project's actual Keil object files.
    cpu_args = ["--target=arm-arm-none-eabi", "-mcpu=cortex-m7",
                "-mfpu=fpv5-d16", "-mfloat-abi=hard"]
    cad = target.find("./TargetOption/TargetArmAds/Cads")
    if cad.findtext("vShortEn") == "1":
        cpu_args.append("-fshort-enums")
    if cad.findtext("vShortWch") == "1":
        cpu_args.append("-fshort-wchar")
    misc = shlex.split(controls.findtext("MiscControls", ""))
    resource = Path(subprocess.check_output([str(compiler), "-print-resource-dir"], text=True).strip())
    system_includes = [compiler.parent.parent / "include", resource / "include"]
    for path in system_includes:
        if not path.is_dir():
            raise SystemExit(f"Compiler headers not found: {path}")
    common = cpu_args + misc + ["-D" + d for d in defines] + ["-U" + d for d in undefines]
    common += ["-I" + p.as_posix() for p in includes]
    entries = []
    for group in target.findall("./Groups/Group"):
        if not enabled(group.find("GroupOption")):
            continue
        for file in group.findall("./Files/File"):
            if file.findtext("FileType") != "1" or not enabled(file.find("FileOption")):
                continue
            path = (PROJECT.parent / file.findtext("FilePath").replace("\\", "/")).resolve()
            if not path.is_file():
                raise SystemExit(f"Source missing: {path}")
            cpp = path.suffix.lower() in {".cpp", ".cxx", ".cc"}
            language = ["-std=c++11", "-x", "c++"] if cpp else ["-std=c99", "-x", "c"]
            entries.append({"directory": PROJECT.parent.as_posix(), "file": path.as_posix(),
                            "arguments": [compiler.as_posix()] + language + common + ["-c", path.as_posix()]})
    write_json(ROOT / ".vscode/compile_commands.json", entries)
    configuration = {
        "name": "Keil " + options.target,
        "compilerPath": compiler.as_posix(), "compilerArgs": cpu_args + misc,
        "intelliSenseMode": "windows-clang-arm", "cStandard": "c99", "cppStandard": "c++11",
        "includePath": [workspace_path(p) for p in includes + system_includes],
        "defines": defines,
        "compileCommands": "${workspaceFolder}/.vscode/compile_commands.json",
        "browse": {"path": [workspace_path(p) for p in includes], "limitSymbolsToIncludedHeaders": True},
    }
    config_path = ROOT / ".vscode/c_cpp_properties.json"
    previous = json.loads(config_path.read_text(encoding="utf-8-sig")) if config_path.exists() else {}
    configurations = [c for c in previous.get("configurations", []) if c.get("name") != configuration["name"]]
    write_json(config_path, {"configurations": [configuration] + configurations, "version": 4})
    print(f"Keil target: {options.target}; {len(includes)} include paths, {len(defines)} defines, {len(entries)} source files")
    if options.check:
        def check(entry):
            args = list(entry["arguments"])
            args[-2] = "-fsyntax-only"
            result = subprocess.run(args, cwd=entry["directory"], capture_output=True, text=True, errors="replace")
            return entry["file"], result.returncode, result.stderr
        with ThreadPoolExecutor(max_workers=4) as executor:
            results = list(executor.map(check, entries))
        failed = [(p, error) for p, code, error in results if code]
        for path, error in failed:
            print(path + "\n" + error)
        print(f"Arm syntax check: {len(results)-len(failed)}/{len(results)} passed")
        if failed:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
