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
		-DSHIELD="5deg_left nice_view" \
		-DZMK_CONFIG=/src/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)"
	@cp build/left/zephyr/zmk.uf2 left.uf2
	@echo "→ left.uf2"

right: docker
	$(DOCKER_RUN) $(WEST_BUILD) -d /src/build/right -- \
		-DSHIELD="5deg_right nice_view" \
		-DZMK_CONFIG=/src/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)"
	@cp build/right/zephyr/zmk.uf2 right.uf2
	@echo "→ right.uf2"

# ── Clean build ────────────────────────────────────────────────────

clean:
	rm -rf build

# ── Interactive shell ──────────────────────────────────────────────

shell: docker
	docker run --rm -it -v $(CURDIR):/src -w /src $(DOCKER_IMAGE)
