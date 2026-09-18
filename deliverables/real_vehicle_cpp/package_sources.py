"""Create and verify a patched copy; never modify the user's original archive."""
import hashlib
import copy
import io
import json
from pathlib import Path
import shutil
import zipfile

HERE = Path(__file__).resolve().parent
SOURCE = Path('D:/WechatFiles/xwechat_files/wxid_2ohckavcfy9722_a041/msg/file/2026-09/实车实验.zip')
TARGET = HERE.parent / '实车实验_已补充PP_LQR.zip'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def rewrite(source, destination, replacements):
    missing = set(replacements) - set(source.namelist())
    assert not missing, missing
    for info in source.infolist():
        output_info = copy.copy(info)  # zipfile mutates offsets when writing.
        if info.filename in replacements:
            destination.writestr(output_info, replacements[info.filename])
        else:
            with source.open(info) as reader, destination.open(output_info, 'w') as writer:
                shutil.copyfileobj(reader, writer)


def verify(original, changed, replacements, extra=()):
    assert set(changed.namelist()) == set(original.namelist()) | set(extra)
    assert changed.testzip() is None
    for name in original.namelist():
        expected = replacements.get(name)
        if expected is None:
            expected = original.read(name)
        assert sha(changed.read(name)) == sha(expected), name


def main():
    original_hash = sha(SOURCE.read_bytes())
    code = {name: (HERE / f'{name}_tracking_controller.cpp').read_bytes() for name in ('pp', 'lqr')}
    with zipfile.ZipFile(SOURCE) as original:
        inner_name = next(n for n in original.namelist() if n.endswith('/sydl_smartcar_ws.zip'))
        inner_bytes = original.read(inner_name)
        result = io.BytesIO()
        prefix = 'sydl_smartcar_ws/src/driverless_package/driverless/src/tracking_controller/'
        changes = {prefix+f'{name}_tracking_controller.cpp': data for name, data in code.items()}
        with zipfile.ZipFile(io.BytesIO(inner_bytes)) as inner:
            with zipfile.ZipFile(result, 'w') as updated:
                rewrite(inner, updated, changes)
            with zipfile.ZipFile(io.BytesIO(result.getvalue())) as updated:
                verify(inner, updated, changes)
        outer_changes = {inner_name: result.getvalue()}
        prefix = '实车实验/VDM课程_待修改补充代码/'
        for name, data in code.items():
            for suffix in (' student.cpp', '.cpp'):
                outer_changes[prefix+f'{name}_tracking_controller'+suffix] = data
        readme_name = '实车实验/本次补充说明.md'
        with zipfile.ZipFile(TARGET, 'w') as updated:
            rewrite(original, updated, outer_changes)
            updated.writestr(readme_name, (HERE/'README.md').read_bytes())
        with zipfile.ZipFile(TARGET) as updated:
            verify(original, updated, outer_changes, (readme_name,))
    assert sha(SOURCE.read_bytes()) == original_hash
    report = {'archive': TARGET.name, 'original_unchanged_sha256': original_hash,
              'new_archive_sha256': sha(TARGET.read_bytes()),
              'updated_outer_cpp_count': 4, 'updated_inner_cpp_count': 2,
              'other_entries_byte_identical': True,
              'source_sha256': {name: sha(data) for name, data in code.items()}}
    (HERE/'package_verification.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
