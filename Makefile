PYTHON = python3
MAP ?= maps/easy/01_linear_path.txt

.PHONY: install run debug clean lint

install:
	$(PYTHON) -m pip install -r requirements.txt

run:
	$(PYTHON) main.py $(MAP)

debug:
	$(PYTHON) -m pdb main.py $(MAP)

clean:
	find . -type d -name "__pycache__" -prune -exec rm -rf {} +
	rm -rf .mypy_cache .pytest_cache .ruff_cache
	find . -type f -name "*.pyc" -delete

lint:
	$(PYTHON) -m flake8 .
	$(PYTHON) -m mypy .
