"""Tables for the scenario path editor: a trajectory from a CSV, TSV or Excel (.xlsx) file.

read_table(name, data) -> {'columns': [...], 'rows': [[...], ...]} with the first row as the
header when it is not numeric (otherwise columns are named c1, c2, ...). Cells are numbers when
they parse as numbers (a decimal comma is accepted), else strings. The .xlsx reader only needs
the standard library: the first worksheet's cells (shared strings and inline values).
"""
import csv
import io
import re
import zipfile
import xml.etree.ElementTree as ET

MAX_ROWS = 100000
NS = {'m': 'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}


def _cell(v):
    if isinstance(v, (int, float)):
        return v
    t = str(v).strip()
    try:
        return float(t.replace(',', '.')) if re.fullmatch(r'[-+]?\d+(?:[.,]\d*)?(?:[eE][-+]?\d+)?', t) else t
    except ValueError:
        return t


def _text_rows(text: str) -> list:
    sample = text[:4096]
    if '\t' in sample:
        delim = '\t'
    elif ';' in sample and sample.count(';') >= sample.count(','):
        delim = ';'                     # Excel CSV with a decimal comma
    else:
        delim = ','
    return [row for row in csv.reader(io.StringIO(text), delimiter=delim) if any(c.strip() for c in row)]


def _col_index(ref: str) -> int:
    n = 0
    for ch in re.match(r'[A-Z]+', ref).group(0):
        n = n * 26 + ord(ch) - 64
    return n - 1


def _xlsx_rows(data: bytes) -> list:
    z = zipfile.ZipFile(io.BytesIO(data))
    shared = []
    if 'xl/sharedStrings.xml' in z.namelist():
        for si in ET.fromstring(z.read('xl/sharedStrings.xml')).findall('m:si', NS):
            shared.append(''.join(t.text or '' for t in si.iter(f'{{{NS["m"]}}}t')))
    sheets = sorted(n for n in z.namelist() if re.fullmatch(r'xl/worksheets/sheet\d+\.xml', n))
    if not sheets:
        raise ValueError('no worksheet in the file')
    rows = []
    for r in ET.fromstring(z.read(sheets[0])).iter(f'{{{NS["m"]}}}row'):
        row = []
        for c in r.findall('m:c', NS):
            i = _col_index(c.get('r', 'A1'))
            v = c.find('m:v', NS)
            if c.get('t') == 's' and v is not None:
                val = shared[int(v.text)]
            elif c.get('t') == 'inlineStr':
                val = ''.join(t.text or '' for t in c.iter(f'{{{NS["m"]}}}t'))
            else:
                val = v.text if v is not None else ''
            row += [''] * (i + 1 - len(row))
            row[i] = val
        if any(str(x).strip() for x in row):
            rows.append(row)
        if len(rows) > MAX_ROWS:
            break
    return rows


def read_table(name: str, data: bytes) -> dict:
    if name.lower().endswith(('.xlsx', '.xlsm')) or data[:2] == b'PK':
        rows = _xlsx_rows(data)
    elif name.lower().endswith('.xls'):
        raise ValueError('old .xls format: save it as .xlsx or CSV')
    else:
        rows = _text_rows(data.decode('utf-8-sig', errors='replace'))
    rows = [[_cell(c) for c in r] for r in rows[:MAX_ROWS + 1]]
    if not rows:
        raise ValueError('empty table')
    width = max(len(r) for r in rows)
    rows = [r + [''] * (width - len(r)) for r in rows]
    if any(isinstance(c, str) and c for c in rows[0]):
        columns = [str(c) or f'c{i + 1}' for i, c in enumerate(rows[0])]
        rows = rows[1:]
    else:
        columns = [f'c{i + 1}' for i in range(width)]
    return {'columns': columns, 'rows': rows}
