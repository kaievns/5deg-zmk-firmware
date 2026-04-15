DOCKER_IMAGE := 5deg-zmk
DOCKER_RUN   := docker run --rm -t -v $(CURDIR):/src $(DOCKER_IMAGE)
BOARD        ?= nice_nano@2.0.0/nrf52840/zmk
EXTRA_MODULES := /src/modules/pmw3610;/src/modules/trackball-processors;/src/modules/mouse-layer
WEST_BUILD    = west build -s zmk/app -b $(BOARD)

.PHONY: all left right clean shell docker reset-left reset-right reset

all: left right

# ── Docker image ───────────────────────────────────────────────────

docker:
	docker build -t $(DOCKER_IMAGE) .

# ── Build firmware ─────────────────────────────────────────────────

left: docker
	$(DOCKER_RUN) $(WEST_BUILD) -d /src/build/left -- \
		-DSHIELD="5deg_left" \
		-DZMK_CONFIG=/src/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)"
	@cp build/left/zephyr/zmk.uf2 left.uf2
	@echo "→ left.uf2"

right: docker
	$(DOCKER_RUN) $(WEST_BUILD) -d /src/build/right -- \
		-DSHIELD="5deg_right" \
		-DZMK_CONFIG=/src/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)"
	@cp build/right/zephyr/zmk.uf2 right.uf2
	@echo "→ right.uf2"

# ── Debug build (USB logging on left) ─────────────────────────────

log-left: docker
	rm -rf build/log-left
	$(DOCKER_RUN) $(WEST_BUILD) -d /src/build/log-left -S zmk-usb-logging -- \
		-DSHIELD="5deg_left nice_view" \
		-DZMK_CONFIG=/src/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)" \
		-DCONFIG_LOG_BUFFER_SIZE=65536 \
		-DCONFIG_DISPLAY_LOG_LEVEL_DBG=y \
		-DCONFIG_SPI_LOG_LEVEL_DBG=y \
		-DCONFIG_LOG_PROCESS_THREAD_STARTUP_DELAY_MS=3000
	@cp build/log-left/zephyr/zmk.uf2 log-left.uf2
	@echo "→ log-left.uf2"

# ── Debug build (USB logging on right) ────────────────────────────

log-right: docker
	rm -rf build/log-right
	$(DOCKER_RUN) $(WEST_BUILD) -d /src/build/log-right -S zmk-usb-logging -- \
		-DSHIELD="5deg_right nice_view" \
		-DZMK_CONFIG=/src/config \
		-DZMK_EXTRA_MODULES="$(EXTRA_MODULES)" \
		-DCONFIG_LOG_BUFFER_SIZE=65536 \
		-DCONFIG_PMW3610_LOG_LEVEL_DBG=y \
		-DCONFIG_LOG_PROCESS_THREAD_STARTUP_DELAY_MS=3000
	@cp build/log-right/zephyr/zmk.uf2 log-right.uf2
	@echo "→ log-right.uf2"

# ── Reset firmware ────────────────────────────────────────────────

reset: docker
	$(DOCKER_RUN) $(WEST_BUILD) -d /src/build/reset -- \
		-DSHIELD=settings_reset \
		-DZMK_CONFIG=/src/config
	@cp build/reset/zephyr/zmk.uf2 reset.uf2
	@echo "→ reset.uf2"

# ── Clean build ────────────────────────────────────────────────────

clean:
	rm -rf build && rm -rf *.uf2

# ── Interactive shell ──────────────────────────────────────────────

shell: docker
	docker run --rm -it -v $(CURDIR):/src -w /src $(DOCKER_IMAGE)
