from gc import collect


def garbage():
    while True:
        collect()
        yield