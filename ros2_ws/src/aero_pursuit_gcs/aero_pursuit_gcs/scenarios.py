"""Scenarios in the GCS: files in <repo>/scenarios (list, read, write, delete) and
preview through scenario_tool (the same C++ code the scenario_runner executes,
so the drawn path is what will be flown).
"""

import asyncio
import json
import logging
import os
import re
from pathlib import Path

import yaml

log = logging.getLogger('gcs.scenarios')

NAME = re.compile(r'^[A-Za-z0-9_\-]{1,64}$')


class _Dumper(yaml.SafeDumper):
    """Short lists of numbers inline ([x, y, z]); everything else as a block."""


def _represent_list(dumper, data):
    flow = len(data) <= 6 and all(isinstance(v, (int, float, str, bool)) for v in data)
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=flow)


_Dumper.add_representer(list, _represent_list)


def dump_yaml(doc: dict) -> str:
    return yaml.dump(doc, Dumper=_Dumper, sort_keys=False, allow_unicode=True, width=100)


class ScenarioStore:
    # Subfolders listed with the root (studies/ holds study files, not scenarios).
    SKIP_DIRS = {'studies'}

    def __init__(self, workspace: Path, backend: str = 'gazebo'):
        self.workspace = Path(workspace).expanduser()
        self.backend = backend
        env = os.environ.get('AERO_SCENARIO_DIR')
        self.dir = Path(env).expanduser() if env else self.workspace.parent / 'scenarios'

    # -- arquivos --------------------------------------------------------------------
    def _path(self, file: str) -> Path:
        """<name>.yaml or <folder>/<name>.yaml inside the scenario directory."""
        parts = Path(file).parts
        if len(parts) > 2 or (len(parts) == 2 and not NAME.match(parts[0])):
            raise ValueError(f'invalid path: {file!r} (at most one subfolder)')
        name = parts[-1] if parts else ''
        if not name.endswith('.yaml'):
            name += '.yaml'
        if not NAME.match(name[:-5]):
            raise ValueError(f'invalid file name: {file!r} (letters, digits, _ or -)')
        return self.dir.joinpath(*parts[:-1], name)

    def _rel(self, p: Path) -> str:
        return str(p.relative_to(self.dir))

    def list(self):
        out = []
        files = sorted(self.dir.glob('*.yaml')) + sorted(
            p for p in self.dir.glob('*/*.yaml') if p.parent.name not in self.SKIP_DIRS)
        for p in files:
            try:
                doc = yaml.safe_load(p.read_text()) or {}
            except (OSError, yaml.YAMLError) as e:
                doc = {'description': f'invalid YAML: {e}'}
            if not isinstance(doc, dict) or 'scenarios' in doc or 'variants' in doc:
                continue        # study/tuning files (tuning/tune_*.yaml)
            out.append({'file': self._rel(p), 'name': doc.get('name', p.stem),
                        'description': doc.get('description', ''),
                        'mtime': p.stat().st_mtime,
                        })
        return out

    def read(self, file: str) -> dict:
        p = self._path(file)
        text = p.read_text()
        return {'file': self._rel(p), 'path': str(p), 'yaml': text, 'doc': yaml.safe_load(text) or {}}

    def write(self, file: str, doc=None, text=None) -> dict:
        """Writes the document (form) or the YAML text as it came (keeps comments)."""
        p = self._path(file)
        if text is None:
            text = dump_yaml(doc)
        else:
            yaml.safe_load(text)    # rejects malformed YAML before overwriting
        self.dir.mkdir(parents=True, exist_ok=True)
        tmp = p.with_suffix('.tmp')
        tmp.write_text(text)
        tmp.replace(p)
        return self.read(self._rel(p))

    def delete(self, file: str):
        self._path(file).unlink(missing_ok=True)

    def path(self, file: str) -> Path:
        return self._path(file)

    # -- scenario_tool ---------------------------------------------------------------
    def tool(self) -> Path:
        candidates = [self.workspace / 'install' / 'aero_pursuit' / 'lib' / 'aero_pursuit' / 'scenario_tool']
        prefix = os.environ.get('AMENT_PREFIX_PATH', '')
        candidates += [Path(p) / 'lib' / 'aero_pursuit' / 'scenario_tool' for p in prefix.split(':') if p]
        for c in candidates:
            if c.is_file() and os.access(c, os.X_OK):
                return c
        raise FileNotFoundError('scenario_tool not found: build aero_pursuit')

    async def run_tool(self, command: str, text: str, *args) -> dict:
        proc = await asyncio.create_subprocess_exec(
            str(self.tool()), command, '-', *args,
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE)
        out, err = await asyncio.wait_for(proc.communicate(text.encode()), 20)
        try:
            return json.loads(out.decode() or '{}')
        except ValueError:
            return {'ok': False, 'error': (err or out).decode(errors='replace')[:2000]}

    async def preview(self, doc=None, text=None, horizon=0.0) -> dict:
        text = text if text is not None else dump_yaml(doc)
        args = ['--dt', '0.25'] + (['--horizon', str(horizon)] if horizon else [])
        res = await self.run_tool('preview', text, *args)
        res['yaml'] = text
        return res

