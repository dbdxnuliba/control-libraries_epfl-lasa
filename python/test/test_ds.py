import unittest
from dynamical_systems import CartesianDS, DYNAMICAL_SYSTEM
import state_representation as sr

DS_METHOD_EXPECTS = [
    'is_compatible',
    'evaluate',
    'get_base_frame',
    'set_base_frame',
    'get_parameter',
    'get_parameters',
    'get_parameter_list',
    'set_parameter'
]

class TestDynamicalSystems(unittest.TestCase):

    def test_callable_methods(self):
        methods = [m for m in dir(CartesianDS) if callable(getattr(CartesianDS, m))]
        for expected in DS_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_construct(self):
        test = CartesianDS(DYNAMICAL_SYSTEM.NONE)
        print(test.get_parameter("attractor"))
        print(test.get_parameter_value("attractor"))
        print(test.get_parameter("gain"))
        print(test.get_base_frame())
        print(test.is_compatible(sr.CartesianState("test")))
        print(test.get_parameters())
        print(test.get_parameter_list())
        test.set_parameter(sr.Parameter("attractor", sr.CartesianState.Identity("teeest", "test"), sr.StateType.PARAMETER_CARTESIANSTATE))
        print(test.get_parameters())
        print(test.get_base_frame())
        print(test.is_compatible(sr.CartesianState("test")))


if __name__ == '__main__':
    unittest.main()
