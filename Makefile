DOCKER_IMAGE := 5deg-zmk
DOCKER_RUN   := docker run --rm -t -v $(CURDIR):/src $(DOCKER_IMAGE)
BOARD        ?= nice_nano_v2
EXTRA_MODULES := /src/modules/drivers;/src/modules/behaviors
WEST_BUILD    = west build -s zmk/app -b $(BOARD)

.PHONY: all left right clean shell docker reset-left reset-right reset

all: left right

# ── Docker image ───────────────────────────────────────────────────

docker:
	docker build -t $(DOCKER_IMAGE) .

# ── Build firmware ─────────────────────────────────────────────────

left: docker
	$(DOCKER_RUN) $(WEST_BUILD) -d /src/build/left -- \
		-DSHIELD=5deg_left \
		-DZMK_CONFIG=/src/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)"
	@cp build/left/zephyr/zmk.uf2 build/left.uf2
	@echo "→ build/left.uf2"

right: docker
	$(DOCKER_RUN) $(WEST_BUILD) -d /src/build/right -- \
		-DSHIELD=5deg_right \
		-DZMK_CONFIG=/src/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)"
	@cp build/right/zephyr/zmk.uf2 build/right.uf2
	@echo "→ build/right.uf2"

# ── Settings reset builds (clears BT bonds + saved state) ─────────

RESET_FLAG := -DCONFIG_ZMK_SETTINGS_RESET_ON_START=y

reset-left: docker
	rm -rf build/reset-left
	$(DOCKER_RUN) $(WEST_BUILD) -d /src/build/reset-left -- \
		-DSHIELD=5deg_left \
		-DZMK_CONFIG=/src/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)" \
		$(RESET_FLAG)
	@cp build/reset-left/zephyr/zmk.uf2 build/reset-left.uf2
	@echo "→ build/reset-left.uf2 (flash, then reflash with normal build!)"

reset-right: docker
	rm -rf build/reset-right
	$(DOCKER_RUN) $(WEST_BUILD) -d /src/build/reset-right -- \
		-DSHIELD=5deg_right \
		-DZMK_CONFIG=/src/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)" \
		$(RESET_FLAG)
	@cp build/reset-right/zephyr/zmk.uf2 build/reset-right.uf2
	@echo "→ build/reset-right.uf2 (flash, then reflash with normal build!)"

reset: reset-left reset-right

# ── Clean build ────────────────────────────────────────────────────

clean:
	rm -rf build

# ── Interactive shell ──────────────────────────────────────────────

shell: docker
	docker run --rm -it -v $(CURDIR):/src -w /src $(DOCKER_IMAGE)
