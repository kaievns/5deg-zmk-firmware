DOCKER_IMAGE := 5deg-zmk
DOCKER_RUN   := docker run --rm -v $(CURDIR):/workspace -w /workspace $(DOCKER_IMAGE)
BOARD        ?= nice_nano_v2
EXTRA_MODULES := /workspace/modules/drivers;/workspace/modules/behaviors
WEST_BUILD    = west build -s zmk/app -b $(BOARD)

.PHONY: all left right clean flash-left flash-right shell docker setup

all: left right

# ── Docker image ───────────────────────────────────────────────────

docker:
	docker build -t $(DOCKER_IMAGE) .

# ── Build firmware ─────────────────────────────────────────────────

left: docker
	$(DOCKER_RUN) $(WEST_BUILD) -d build/left -- \
		-DSHIELD=5deg_left \
		-DZMK_CONFIG=/workspace/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)"
	mkdir -p output
	docker run --rm -v $(CURDIR):/workspace $(DOCKER_IMAGE) \
		cp build/left/zephyr/zmk.uf2 output/5deg_left.uf2
	@echo "→ output/5deg_left.uf2"

right: docker
	$(DOCKER_RUN) $(WEST_BUILD) -d build/right -- \
		-DSHIELD=5deg_right \
		-DZMK_CONFIG=/workspace/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)"
	mkdir -p output
	docker run --rm -v $(CURDIR):/workspace $(DOCKER_IMAGE) \
		cp build/right/zephyr/zmk.uf2 output/5deg_right.uf2
	@echo "→ output/5deg_right.uf2"

# ── Clean build ────────────────────────────────────────────────────

clean:
	rm -rf build output

# ── Flash (host-side, no docker) ───────────────────────────────────

flash-left:
	./scripts/flash.sh left

flash-right:
	./scripts/flash.sh right

# ── Interactive shell ──────────────────────────────────────────────

shell: docker
	docker run --rm -it -v $(CURDIR):/workspace -w /workspace $(DOCKER_IMAGE)

# ── Native (no docker) setup ──────────────────────────────────────

setup:
	./scripts/setup.sh
