import structlog

import aeris.core.logging as aeris_logging
from aeris.core.logging import bind_run_context, configure_logging, get_logger


def _capture_logs():
    # structlog.testing.capture_logs() disables ALL configured processors
    # (including contextvars merging) unless told to keep them — pass the
    # one AERIS actually relies on for run_id/t_sim propagation.
    return structlog.testing.capture_logs(processors=[structlog.contextvars.merge_contextvars])


def test_configure_logging_is_idempotent():
    configure_logging()
    configure_logging()  # must not raise or reconfigure destructively


def test_get_logger_returns_a_logger_without_explicit_configure():
    logger = get_logger()
    assert logger is not None


def test_get_logger_auto_configures_if_nothing_configured_it_yet(monkeypatch):
    # Simulate a fresh process where configure_logging() was never called.
    monkeypatch.setattr(aeris_logging, "_CONFIGURED", False)
    logger = get_logger()
    assert logger is not None
    assert aeris_logging._CONFIGURED is True


def test_get_logger_binds_initial_context():
    with _capture_logs() as captured:
        get_logger(component="safety").info("hello")
    assert captured[0]["component"] == "safety"
    assert captured[0]["event"] == "hello"


def test_bind_run_context_adds_run_id_to_log_lines():
    with _capture_logs() as captured, bind_run_context(run_id="run-123"):
        get_logger().info("tick")
    assert captured[0]["run_id"] == "run-123"


def test_bind_run_context_adds_t_sim_when_given():
    with _capture_logs() as captured, bind_run_context(run_id="run-1", t_sim_s=12.5):
        get_logger().info("tick")
    assert captured[0]["t_sim_s"] == 12.5


def test_bind_run_context_does_not_leak_outside_the_with_block():
    with bind_run_context(run_id="run-1"):
        pass
    with _capture_logs() as captured:
        get_logger().info("after")
    assert "run_id" not in captured[0]


def test_bind_run_context_restores_previous_context_on_exit():
    with _capture_logs() as captured, bind_run_context(run_id="outer"):
        with bind_run_context(run_id="inner"):
            get_logger().info("inner-tick")
        get_logger().info("outer-tick")
    assert captured[0]["run_id"] == "inner"
    assert captured[1]["run_id"] == "outer"
