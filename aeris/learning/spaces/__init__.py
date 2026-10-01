"""Observation/action spaces shared between FastSim training and Tier H
deployment (spec §27.6). Nothing here may import a simulator: the same code
must run on a real vehicle's data. Enforced by an import-linter contract."""
