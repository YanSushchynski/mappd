


def test_gen(name, **kwargs):
    print(name)
    tags = kwargs.pop("tags", default = [])
    main_name = kwargs.pop("name", default = name + "_")
    excluded_pkgs = kwargs.pop("excluded_pkgs", default = [])
    native.cc_test(name = name, **kwargs)