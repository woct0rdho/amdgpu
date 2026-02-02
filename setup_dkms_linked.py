#!/usr/bin/env python3

import os
import shutil

SOURCE_MAP_FILE = "drivers/gpu/drm/amd/dkms/sources"
DEST_DIR = "amdgpu-dkms"


def link_tree(src_dir, dest_dir):
    """
    Recursively replicates directory structure and symlinks files.
    """
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)

    for item in os.listdir(src_dir):
        src_path = os.path.join(src_dir, item)
        dest_path = os.path.join(dest_dir, item)
        if os.path.isdir(src_path):
            link_tree(src_path, dest_path)
        else:
            # Create symlink pointing to absolute path of source
            if os.path.lexists(dest_path):
                os.remove(dest_path)
            os.symlink(os.path.abspath(src_path), dest_path)


def main():
    if os.path.exists(DEST_DIR):
        shutil.rmtree(DEST_DIR)
    os.makedirs(DEST_DIR)

    with open(SOURCE_MAP_FILE, 'r') as f:
        lines = f.readlines()

    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue

        parts = line.split()
        if len(parts) < 2:
            continue

        src_path = parts[0]
        dest_path = parts[1]
        if dest_path == ".":
            base_name = os.path.basename(src_path)
            full_dest = os.path.join(DEST_DIR, base_name)
            if os.path.isdir(src_path):
                 link_tree(src_path, full_dest)
            else:
                 if os.path.lexists(full_dest):
                    os.remove(full_dest)
                 os.symlink(os.path.abspath(src_path), full_dest)
        else:
            if dest_path.endswith('/'):
                 full_dest_dir = os.path.join(DEST_DIR, dest_path)
                 if not os.path.exists(full_dest_dir):
                     os.makedirs(full_dest_dir)

                 if os.path.isdir(src_path):
                     base_name = os.path.basename(src_path)
                     full_dest = os.path.join(full_dest_dir, base_name)
                     link_tree(src_path, full_dest)
                 else:
                     base_name = os.path.basename(src_path)
                     full_dest = os.path.join(full_dest_dir, base_name)
                     if os.path.lexists(full_dest):
                        os.remove(full_dest)
                     os.symlink(os.path.abspath(src_path), full_dest)
            else:
                 full_dest = os.path.join(DEST_DIR, dest_path)
                 parent = os.path.dirname(full_dest)
                 if not os.path.exists(parent):
                     os.makedirs(parent)

                 if os.path.lexists(full_dest):
                    os.remove(full_dest)
                 os.symlink(os.path.abspath(src_path), full_dest)

    # Post-processing: Link Makefile and dkms.conf to root
    # They should be in amdgpu-dkms/amd/dkms/

    # We link them to the repo location directly to be consistent
    repo_makefile = os.path.abspath("drivers/gpu/drm/amd/dkms/Makefile")
    dst_makefile = os.path.join(DEST_DIR, "Makefile")
    if os.path.exists(repo_makefile):
        if os.path.lexists(dst_makefile):
             os.remove(dst_makefile)
        os.symlink(repo_makefile, dst_makefile)
        print(f"Linked {dst_makefile} -> {repo_makefile}")

    repo_dkms_conf = os.path.abspath("drivers/gpu/drm/amd/dkms/dkms.conf")
    dst_dkms_conf = os.path.join(DEST_DIR, "dkms.conf")
    if os.path.exists(repo_dkms_conf):
        if os.path.lexists(dst_dkms_conf):
             os.remove(dst_dkms_conf)
        os.symlink(repo_dkms_conf, dst_dkms_conf)
        print(f"Linked {dst_dkms_conf} -> {repo_dkms_conf}")

    print(f"DKMS source tree (linked) created at {os.path.abspath(DEST_DIR)}")
    print("Note: Files modified by the build process will lose their synchronization with the repository after the first build.")


if __name__ == "__main__":
    main()
