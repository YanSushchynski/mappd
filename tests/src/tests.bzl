def test_gen(name, **kwargs):
    """
        Macro for generating rules      
    """
    native.cc_test(name = name, **kwargs)