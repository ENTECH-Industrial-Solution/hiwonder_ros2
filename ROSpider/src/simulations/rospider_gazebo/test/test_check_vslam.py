import pathlib
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
import check_vslam  # noqa: E402


def test_export_failure_is_reported(tmp_path, capsys):
    def missing(*args, **kwargs):
        raise FileNotFoundError('rtabmap-export')
    assert check_vslam.export_grid('x.db', str(tmp_path), run=missing) is None
    assert 'rtabmap-export' in capsys.readouterr().out

    def failing(*args, **kwargs):
        raise subprocess.CalledProcessError(1, 'rtabmap-export', output='boom')
    assert check_vslam.export_grid('x.db', str(tmp_path), run=failing) is None


def test_database_still_open_is_refused(tmp_path, monkeypatch, capsys):
    db = tmp_path / 'mine.db'
    db.write_bytes(b'')
    monkeypatch.setattr(check_vslam.vslam_check, 'db_in_use', lambda path, proc_root='/proc': True)
    monkeypatch.setattr(check_vslam, 'reference_path', lambda: str(tmp_path / 'ref.yaml'))
    assert check_vslam.main([str(db)]) == 1
    assert 'Ctrl+C' in capsys.readouterr().out
