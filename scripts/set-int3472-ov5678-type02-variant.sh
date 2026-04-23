#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./set-int3472-ov5678-type02-variant.sh <power-enable|handshake|privacy-led>

This rewrites int3472-gc5035-skylake.diff for the OVTI5678 type 0x02 mapping.
EOF
}

if [[ $# -ne 1 ]]; then
  usage
  exit 2
fi

variant="$1"
case "$variant" in
  power-enable)
    header="166a167,209"
    ov_block=$'> \t{\t/* Map 0x02 to DVDD regulator for OVTI5678 */\n> \t\t.hid = "OVTI5678",\n> \t\t.type_from = 0x02,\n> \t\t.type_to = INT3472_GPIO_TYPE_POWER_ENABLE,\n> \t\t.con_id = "dvdd",\n> \t},'
    ;;
  handshake)
    header="166a167,210"
    ov_block=$'> \t{\t/* Map 0x02 to HANDSHAKE for OVTI5678 */\n> \t\t.hid = "OVTI5678",\n> \t\t.type_from = 0x02,\n> \t\t.type_to = INT3472_GPIO_TYPE_HANDSHAKE,\n> \t\t.con_id = "dvdd",\n> \t\t.enable_time_us = 25 * USEC_PER_MSEC,\n> \t},'
    ;;
  privacy-led)
    header="166a167,209"
    ov_block=$'> \t{\t/* Map 0x02 to PRIVACY_LED for OVTI5678 */\n> \t\t.hid = "OVTI5678",\n> \t\t.type_from = 0x02,\n> \t\t.type_to = INT3472_GPIO_TYPE_PRIVACY_LED,\n> \t\t.con_id = "privacy-led",\n> \t},'
    ;;
  *)
    usage
    exit 2
    ;;
esac

cat > int3472-gc5035-skylake.diff <<EOF
${header}
> 	/* --- ADDED: Lenovo IdeaPad Duet 5 GC5035 Quirks --- */
> 	{	/* Map 0x02 to DVDD regulator */
> 		.hid = "GCTI5035",
> 		.type_from = 0x02,
> 		.type_to = INT3472_GPIO_TYPE_POWER_ENABLE,
> 		.con_id = "dvdd",
> 	},
${ov_block}
> 	{	/* Map 0x08 to standard Powerdown for OVTI5678 */
> 		.hid = "OVTI5678",
> 		.type_from = 0x08,
> 		.type_to = INT3472_GPIO_TYPE_POWERDOWN,
> 		.con_id = "powerdown",
> 	},
> 	{	/* Map 0x10 to standard Reset for OVTI5678 */
> 		.hid = "OVTI5678",
> 		.type_from = 0x10,
> 		.type_to = INT3472_GPIO_TYPE_RESET,
> 		.con_id = "reset",
> 	},
> 	{	/* Map 0x08 to standard Powerdown */
> 		.hid = "GCTI5035",
> 		.type_from = 0x08,
> 		.type_to = INT3472_GPIO_TYPE_POWERDOWN,
> 		.con_id = "powerdown",
> 	},
> 	{	/* Map 0x10 to standard Reset */
> 		.hid = "GCTI5035",
> 		.type_from = 0x10,
> 		.type_to = INT3472_GPIO_TYPE_RESET,
> 		.con_id = "reset",
> 	},
> 	{	/* Map legacy reset type to second reset line */
> 		.hid = "GCTI5035",
> 		.type_from = INT3472_GPIO_TYPE_RESET,
> 		.type_to = INT3472_GPIO_TYPE_RESET,
> 		.con_id = "reset2",
> 	},
EOF

echo "Updated int3472-gc5035-skylake.diff variant: ${variant}"
