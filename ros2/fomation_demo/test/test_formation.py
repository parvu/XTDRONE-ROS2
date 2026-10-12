import numpy

from xtdrone_formation_demo.avoid import avoid_controls
from xtdrone_formation_demo.formation_dict import formation_dict_9


def test_avoid_pushes_close_pair_apart_symmetrically():
    a, b = numpy.array([0.0, 0.0, 5.0]), numpy.array([1.0, 0.0, 5.0])
    ca, cb, cc = avoid_controls([a, b, numpy.array([10.0, 10.0, 5.0])])
    assert numpy.linalg.norm(ca) > 0
    numpy.testing.assert_allclose(ca, -cb)
    numpy.testing.assert_allclose(cc, 0)
    # The push is perpendicular to the line between the pair.
    assert abs(ca.dot(b - a)) < 1e-9


def test_avoid_ignores_far_and_unknown():
    out = avoid_controls([numpy.zeros(3), numpy.array([2.0, 0, 0]), None])
    for c in out:
        numpy.testing.assert_allclose(c, 0)


def test_formation_dict_shapes():
    for name, pattern in formation_dict_9.items():
        assert pattern.shape == (3, 8), name


def test_link_matrix_close_neighbours_only():
    import pytest
    comm_model = pytest.importorskip("xtdrone_formation_demo.comm_model")
    positions = [numpy.array([x, y, 1.0]) for x, y in comm_model.UAV_POSITIONS]
    leaders = list(range(len(positions)))
    trans = comm_model.link_matrix(positions, leaders)
    assert trans[0] == []
    for i in range(1, len(positions)):
        assert len(trans[i]) == len(leaders)
        assert trans[i][i] == 0  # no link to itself
    # UAV 1 (1.2, -1.2) hears its closest neighbours but not the far group at (3..5, 6).
    assert sum(trans[1]) >= 1
    assert trans[1][19] == 0
