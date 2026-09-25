from aeris.core import errors as e


def test_all_public_errors_derive_from_aeris_error():
    for name in dir(e):
        obj = getattr(e, name)
        if isinstance(obj, type) and issubclass(obj, BaseException):
            assert issubclass(obj, e.AerisError), f"{name} does not derive from AerisError"


def test_safety_errors_are_safety_errors():
    assert issubclass(e.CommandRejectedError, e.SafetyError)
    assert issubclass(e.UnsafeStateError, e.SafetyError)
    assert issubclass(e.WatchdogTimeoutError, e.SafetyError)


def test_provenance_error_is_distinct_from_config_error():
    assert not issubclass(e.OracleObservationError, e.ConfigError)
    assert issubclass(e.OracleObservationError, e.ProvenanceError)


def test_errors_carry_a_message():
    err = e.CommandRejectedError("velocity exceeds v_max")
    assert "v_max" in str(err)
