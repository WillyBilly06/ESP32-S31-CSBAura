"""Export a git checkout's files with their committed bytes (not the working tree's).

On Windows git usually checks files out with CRLF line endings (core.autocrlf), so copying
the working tree would ship CRLF files that differ from upstream. Here every unchanged file
comes from git's own object (exact upstream bytes); a file our patches changed comes from
the working tree, with its line endings put back the way the committed file has them.
Submodules are walked too.
"""
import os
import subprocess


def _git(repo, *args):
    return subprocess.run(["git", *args], cwd=repo, capture_output=True, text=True, check=True).stdout


def _repos(root):
    """(repo dir, prefix) for root and every submodule, recursively"""
    out = [(root, "")]
    paths = _git(root, "submodule", "foreach", "--recursive", "--quiet", "echo $displaypath").splitlines()
    for p in paths:
        p = p.strip().replace("\\", "/")
        if p:
            out.append((os.path.join(root, p), p + "/"))
    return out


class _Blobs:
    """git cat-file --batch for one repository"""

    def __init__(self, repo):
        self.p = subprocess.Popen(["git", "cat-file", "--batch"], cwd=repo,
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    def get(self, sha):
        self.p.stdin.write(sha.encode() + b"\n")
        self.p.stdin.flush()
        head = self.p.stdout.readline().split()
        if len(head) < 3 or head[1] != b"blob":
            raise RuntimeError(f"not a blob: {sha}")
        data = self.p.stdout.read(int(head[2]))
        self.p.stdout.read(1)
        return data

    def close(self):
        self.p.stdin.close()
        self.p.wait()


def export(root, keep, changed=()):
    """yield (path, bytes, executable) for every committed file of root (and its submodules)
    with keep(path) true; paths in `changed` are taken from the working tree"""
    changed = set(changed)
    for repo, prefix in _repos(root):
        blobs = _Blobs(repo)
        try:
            for line in _git(repo, "ls-files", "-s", "-z").split("\0"):
                if not line:
                    continue
                meta, _, f = line.partition("\t")
                mode, sha, _stage = meta.split()
                path = prefix + f
                if mode == "160000" or not keep(path):
                    continue                    # a submodule entry: its files come with its repo
                committed = blobs.get(sha)
                if path in changed:
                    with open(os.path.join(root, path), "rb") as fh:
                        data = fh.read()
                    if b"\r\n" not in committed:
                        data = data.replace(b"\r\n", b"\n")
                else:
                    data = committed
                yield path, data, mode == "100755"
        finally:
            blobs.close()


def changed_paths(repo):
    """the files that differ from the commit in the working tree (no untracked ones)"""
    return {l.split(None, 1)[1] for l in
            _git(repo, "status", "--porcelain", "--untracked-files=no").splitlines() if l.strip()}
