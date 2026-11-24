"""Remembered parameters: changes made through the GCS are saved (settings['param_overrides'])
and re-applied whenever a node (re)appears, e.g. after restarting the simulation."""

import asyncio
import logging

log = logging.getLogger('gcs.params')

# The pursuit node only has the parameters below since the controllers were removed; remembered
# values of anything else (lqr.*, ibvs.*, mpc.lag.*, ...) are dropped instead of being set on a
# node that accepts undeclared parameters (mpc.params.* is the free-form tab of the MPC).
_PURSUIT = '/pursuit_controller_node'
_KEPT = ('target_distance', 'max_vel', 'coast_timeout',
         'lost_timeout', 'bearing_timeout', 'velocity_timeout', 'target_accel.',
         'mpc.params.')
_FIRST: list = []


def migrate_overrides(ov: dict) -> bool:
    """Drops remembered parameters that no longer exist. Returns True if anything changed."""
    pc = ov.get(_PURSUIT)
    if not pc:
        return False
    stale = [k for k in pc if not k.startswith(_KEPT)]
    for k in stale:
        pc.pop(k)
    return bool(stale)


def apply_order(params: dict) -> list:
    return sorted(params.items(), key=lambda kv: _FIRST.index(kv[0]) if kv[0] in _FIRST else len(_FIRST))


class ParamMemory:
    def __init__(self, server):
        self.server = server
        self.settings = server.settings
        self._applied = set()           # nodes that already got the remembered values
        self._busy = False              # one re-application at a time (it sleeps per node)

    def routes(self) -> dict:
        return {'param.list': self.list, 'param.overrides': lambda m: self.message(),
                'param.forget': self.forget, 'param.remember': self.remember}

    def message(self):
        return {'op': 'param.overrides', 'remember': self.settings['remember_params'],
                'overrides': self.settings['param_overrides']}

    async def list(self, m):
        return await self.server.node.list_params(m['node'])

    async def remember_change(self, node, name, type_name, value):
        """Called after an operator changed a parameter through the GCS."""
        ov = self.settings['param_overrides']
        ov.setdefault(node, {})[name] = {'type': type_name, 'value': value}
        self.settings.set('param_overrides', ov)
        await self.server.broadcast(self.message())

    async def forget(self, m):
        ov = self.settings['param_overrides']
        if m.get('name'):
            ov.get(m['node'], {}).pop(m['name'], None)
            if not ov.get(m['node']):
                ov.pop(m['node'], None)
        elif m.get('node'):
            ov.pop(m['node'], None)
        else:
            ov.clear()
        self.settings.set('param_overrides', ov)
        await self.server.broadcast(self.message())

    async def remember(self, m):
        self.settings.set('remember_params', bool(m.get('on')))
        await self.server.broadcast(self.message())

    async def reapply(self):
        """Re-applies the remembered settings on every node that (re)appeared in the graph."""
        if self._busy:
            return
        self._busy = True
        try:
            await self._reapply()
        finally:
            self._busy = False

    async def _reapply(self):
        node = self.server.node
        ov = self.settings['param_overrides']
        if not ov or node is None:
            return
        if migrate_overrides(ov):
            self.settings.set('param_overrides', ov)
            log.info('old remembered settings migrated to the current parameters (removed controllers)')
        present = set(node.list_nodes())
        self._applied &= present        # node vanished: re-apply when it comes back
        for name in [n for n in ov if n in present and n not in self._applied]:
            self._applied.add(name)
            await asyncio.sleep(1.0)    # lets the node finish declaring its parameters
            failed, dropped = [], []
            for pname, p in apply_order(ov.get(name, {})):
                try:
                    ok, reason = await node.set_param(name, pname, p['type'], p['value'])
                except Exception as e:  # noqa: BLE001 - one parameter must not stop the others
                    ok, reason = False, str(e)
                if ok:
                    continue
                if 'not declared' in str(reason) or 'not found' in str(reason):
                    dropped.append(pname)       # the node no longer has it: stop re-applying it
                else:
                    failed.append(pname)
            if dropped:
                for pname in dropped:
                    ov.get(name, {}).pop(pname, None)
                if not ov.get(name):
                    ov.pop(name, None)
                self.settings.set('param_overrides', ov)
                log.info('forgot remembered parameters %s no longer has: %s', name, ', '.join(dropped))
            log.info('remembered settings reapplied to %s%s', name,
                     f' (failed: {", ".join(failed)})' if failed else '')
