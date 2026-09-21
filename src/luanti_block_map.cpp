#include "luanti_block_map.h"
#include "../../arnis_block.h"
#include <string>
namespace arnis
{
LuantiNode to_luanti_node(
		const Block &block, LuantiGame, const char *facing, bool open, bool top)
{
	const auto id = block.id();
	auto facedir = [&]() -> std::uint8_t {
		if (!facing)
			return static_cast<std::uint8_t>(top ? 20 : 0);
		const std::string direction(facing);
		std::uint8_t d = direction == "east"	? 1
						 : direction == "south" ? 2
						 : direction == "west"	? 3
												: 0;
		return static_cast<std::uint8_t>(d + (top ? 20 : 0));
	};
	auto stair = [&](const char *name) { return LuantiNode{name, facedir()}; };
	auto trapdoor = [&](const char *closed, const char *opened) {
		return LuantiNode{open ? opened : closed, facedir()};
	};
	auto door = [&](const char *bottom, const char *upper) {
		return LuantiNode{top ? upper : bottom, facedir()};
	};
	switch (id) {
	case 0:
		return {"mcl_trees:tree_mangrove", 0};
	case 1:
		return {"air", 0};
	case 2:
		return {"mcl_core:andesite", 0};
	case 3:
		return {"mcl_trees:leaves_birch", 0};
	case 4:
		return {"mcl_trees:tree_birch", 0};
	case 5:
		return {"mcl_colorblocks:concrete_black", 0};
	case 6:
		return {"mcl_blackstone:blackstone", 0};
	case 7:
		return {"mcl_flowers:blue_orchid", 0};
	case 8:
		return {"mcl_colorblocks:hardened_clay_blue", 0};
	case 9:
		return {"mcl_core:brick_block", 0};
	case 10:
		return {"mcl_cauldrons:cauldron", 0};
	case 11:
		return {"mcl_core:stonebrickcarved", 0};
	case 12:
		return {"mcl_walls:cobble", 0};
	case 13:
		return {"mcl_core:cobble", 0};
	case 14:
		return {"mcl_blackstone:blackstone_brick_polished", 0};
	case 15:
		return {"mcl_core:stonebrickcracked", 0};
	case 16:
		return {"mcl_colorblocks:concrete_cyan", 0};
	case 17:
		return stair("mcl_stairs:stair_cobble");
	case 18:
		return {"mcl_nether:magma", 0};
	case 19:
		return {"mcl_trees:wood_dark_oak", 0};
	case 20:
		return {"mcl_deepslate:deepslate_bricks", 0};
	case 21:
		return {"mcl_core:diorite", 0};
	case 22:
		return {"mcl_core:dirt", 0};
	case 23:
		return {"mcl_end:end_bricks", 0};
	case 24:
		return {"mcl_farming:soil_wet", 0};
	case 25:
		return {"mcl_core:glass", 0};
	case 26:
		return {"mcl_nether:glowstone", 0};
	case 27:
		return {"mcl_core:granite", 0};
	case 28:
		return {"mcl_core:dirt_with_grass", 0};
	case 29:
		return {"mcl_flowers:tallgrass", 0};
	case 30:
		return {"mcl_core:gravel", 0};
	case 31:
		return {"mcl_colorblocks:concrete_grey", 0};
	case 32:
		return {"mcl_colorblocks:hardened_clay_grey", 0};
	case 33:
		return {"mcl_colorblocks:hardened_clay_green", 0};
	case 34:
		return {"mcl_wool:green", 0};
	case 35:
		return {"mcl_farming:hay_block", 0};
	case 36:
		return {"mcl_panes:bar_flat", 0};
	case 37:
		return {"mcl_core:ironblock", 0};
	case 38:
		return stair("mcl_stairs:stair_copper_cut");
	case 39:
		return {"mcl_colorblocks:concrete_yellow", 0};
	case 40:
		return {"mcl_core:snow", 0};
	case 41:
		return {"mcl_colorblocks:hardened_clay_light_blue", 0};
	case 42:
		return {"mcl_colorblocks:concrete_silver", 0};
	case 43:
		return {"mcl_lush_caves:moss", 0};
	case 44:
		return {"mcl_core:mossycobble", 0};
	case 45:
		return {"mcl_mud:mud_bricks", 0};
	case 46:
		return {"mcl_nether:nether_brick", 0};
	case 47:
		return {"mcl_nether:netheriteblock", 0};
	case 48:
		return {"mcl_fences:oak_fence", 0};
	case 49:
		return {"mcl_trees:leaves_oak", 0};
	case 50:
		return {"mcl_trees:tree_oak", 0};
	case 51:
		return {"mcl_trees:wood_oak", 0};
	case 52:
		return {"mcl_stairs:slab_oak", 0};
	case 53:
		return {"mcl_colorblocks:hardened_clay_orange", 0};
	case 54:
		return {"mcl_core:podzol", 0};
	case 55:
		return {"mcl_core:andesite_smooth", 0};
	case 56:
		return stair("mcl_stairs:stair_stonebrickmossy");
	case 57:
		return {"mcl_nether:quartz_block", 0};
	case 58:
		return {"mcl_blackstone:blackstone_polished", 0};
	case 59:
		return {"mcl_deepslate:deepslate_polished", 0};
	case 60:
		return {"mcl_core:diorite_smooth", 0};
	case 61:
		return {"mcl_core:granite_smooth", 0};
	case 62:
		return stair("mcl_stairs:stair_mossycobble");
	case 63:
		return stair("mcl_stairs:stair_deepslate_bricks");
	case 64:
		return stair("mcl_stairs:stair_deepslate_polished");
	case 65:
		return {"mcl_nether:quartz_block", 0};
	case 66:
	case 70:
	case 87:
	case 93:
	case 95:
		return {"mcl_core:water_source", 0};
	case 67:
		return {"mcl_flowers:poppy", 0};
	case 68:
		return {"mcl_nether:red_nether_brick", 0};
	case 69:
		return {"mcl_colorblocks:hardened_clay_red", 0};
	case 71:
		return {"mcl_core:sand", 0};
	case 72:
		return {"mcl_core:sandstone", 0};
	case 73:
		return {"mcl_bamboo:scaffolding", 0};
	case 74:
		return {"mcl_nether:quartz_smooth", 0};
	case 75:
		return stair("mcl_stairs:stair_spruce");
	case 76:
		return {"mcl_core:sandstonesmooth2", 0};
	case 77:
		return {"mcl_stairs:slab_stone_double", 0};
	case 78:
		return {"mcl_sponges:sponge", 0};
	case 79:
		return {"mcl_trees:tree_spruce", 0};
	case 80:
		return {"mcl_trees:wood_spruce", 0};
	case 81:
		return {"mcl_stairs:slab_stone", 0};
	case 82:
		return {"mcl_stairs:slab_stonebrick", 0};
	case 83:
		return {"mcl_core:stonebrick", 0};
	case 84:
		return {"mcl_core:stone", 0};
	case 85:
		return {"mcl_colorblocks:hardened_clay", 0};
	case 86:
		return stair("mcl_stairs:stair_dark_oak");
	case 88:
		return {"mcl_colorblocks:concrete_white", 0};
	case 89:
		return {"mcl_flowers:azure_bluet", 0};
	case 90:
		return {"mcl_core:glass_white", 0};
	case 91:
		return {"mcl_colorblocks:hardened_clay_white", 0};
	case 92:
		return {"mcl_wool:white", 0};
	case 94:
		return {"mcl_flowers:dandelion", 0};
	case 96:
		return {"mcl_nether:soul_sand", 0};
	case 97:
		return stair("mcl_stairs:stair_red_nether_brick");
	case 98:
		return {"mcl_walls:sandstone", 0};
	case 99:
		return {"mcl_stairs:slab_sandstonesmooth", 0};
	case 100:
		return {"mcl_colorblocks:concrete_red", 0};
	case 101:
		return trapdoor("mcl_doors:iron_trapdoor", "mcl_doors:iron_trapdoor_open");
	case 104:
		return {"mcl_colorblocks:hardened_clay_yellow", 0};
	case 105:
		return {"mcl_farming:carrot_7", 0};
	case 106:
	case 107:
		return door("mcl_doors:door_dark_oak_b_1", "mcl_doors:door_dark_oak_t_1");
	case 108:
		return {"mcl_farming:potato_4", 0};
	case 109:
		return {"mcl_farming:wheat_7", 0};
	case 110:
		return {"mcl_core:bedrock", 0};
	case 111:
		return {"mcl_core:snowblock", 0};
	case 112:
		return stair("mcl_stairs:stair_andesite");
	case 113:
		return trapdoor("mcl_doors:trapdoor_jungle", "mcl_doors:trapdoor_jungle_open");
	case 114:
		return {"mcl_walls:andesite", 0};
	case 115:
		return {"mcl_walls:stonebrick", 0};
	case 116:
	case 117:
	case 118:
	case 119:
	case 120:
	case 121:
	case 122:
	case 123:
	case 124:
	case 125:
		return {"mcl_minecarts:rail", 0};
	case 126:
		return {"mcl_core:coarse_dirt", 0};
	case 127:
		return {"mcl_core:stone_with_iron", 0};
	case 128:
		return {"mcl_core:stone_with_coal", 0};
	case 129:
		return {"mcl_core:stone_with_gold", 0};
	case 130:
		return {"mcl_copper:stone_with_copper", 0};
	case 131:
		return {"mcl_core:clay", 0};
	case 132:
		return {"mcl_core:grass_path", 0};
	case 134:
		return {"mcl_core:packed_ice", 0};
	case 135:
		return {"mcl_mud:mud", 0};
	case 136:
		return {"mcl_core:deadbush", 0};
	case 137:
		return {"mcl_flowers:double_grass", 0};
	case 138:
		return {"mcl_flowers:double_grass_top", 0};
	case 139:
		return {"mcl_crafting_table:crafting_table", 0};
	case 140:
		return {"mcl_furnaces:furnace", 0};
	case 141:
		return {"mcl_wool:white_carpet", 0};
	case 142:
		return {"mcl_books:bookshelf", 0};
	case 143:
		return {"mcl_trees:wood_oak", 0};
	case 144:
		return stair("mcl_stairs:stair_oak");
	case 145:
		return {"mcl_colorblocks:concrete_orange", 0};
	case 146:
		return {"mcl_colorblocks:concrete_purple", 0};
	case 147:
		return {"mcl_fences:birch_fence_gate", 0};
	case 148:
		return {"mcl_fences:dark_oak_fence_gate", 0};
	case 149:
		return {"mcl_colorblocks:concrete_light_blue", 0};
	case 150:
		return {"mcl_core:stonebrickmossy", 0};
	case 151:
		return {"mcl_deepslate:deepslate", 0};
	case 152:
		return {"mcl_deepslate:tuff", 0};
	case 153:
		return {"mcl_deepslate:deepslate_cobbled", 0};
	case 154:
		return {"mcl_lanterns:lantern_floor", 0};
	case 155:
		return {"mcl_chests:chest", facedir()};
	case 156:
		return {"mcl_buttons:button_stone_off", facedir()};
	case 157:
		return {"mcl_anvils:anvil", 0};
	case 158:
		return {"mcl_noteblock:noteblock", 0};
	case 159:
		return {"mcl_deepslate:deepslatepolishedwall", 0};
	case 160:
		return {"mcl_brewing:stand_000", 0};
	case 161:
	case 162:
	case 165:
	case 166:
	case 167:
	case 168:
	case 294:
	case 295:
		return {"mcl_beds:bed_red_bottom", facedir()};
	case 163:
		return {"mcl_core:glass_black", 0};
	case 164:
		return {"mcl_stairs:slab_andesite_smooth", 0};
	case 169:
		return {"mcl_core:glass_grey", 0};
	case 170:
		return {"mcl_core:glass_silver", 0};
	case 171:
		return {"mcl_core:glass_brown", 0};
	case 172:
		return {"mcl_core:glass", 0};
	case 173:
		return {"mcl_colorblocks:concrete_magenta", 0};
	case 174:
		return {"mcl_colorblocks:concrete_brown", 0};
	case 175:
		return {"mcl_colorblocks:hardened_clay_black", 0};
	case 176:
		return {"mcl_colorblocks:hardened_clay_brown", 0};
	case 177:
		return stair("mcl_stairs:stair_stonebrick");
	case 178:
		return stair("mcl_stairs:stair_mud_brick");
	case 179:
		return stair("mcl_stairs:stair_blackstone_brick_polished");
	case 180:
		return stair("mcl_stairs:stair_brick_block");
	case 181:
		return stair("mcl_stairs:stair_granite_smooth");
	case 182:
		return stair("mcl_stairs:stair_end_bricks");
	case 183:
		return stair("mcl_stairs:stair_diorite_smooth");
	case 184:
		return stair("mcl_stairs:stair_sandstone");
	case 185:
		return stair("mcl_stairs:stair_quartzblock");
	case 186:
		return stair("mcl_stairs:stair_andesite_smooth");
	case 187:
		return stair("mcl_stairs:stair_nether_brick");
	case 188:
		return {"mcl_barrels:barrel_closed", 0};
	case 189:
		return {"mcl_flowers:fern", 0};
	case 190:
		return {"mcl_colorblocks:concrete_lime", 0};
	case 191:
		return {"mcl_colorblocks:concrete_blue", 0};
	case 192:
		return {"mcl_panes:pane_grey_flat", 0};
	case 193:
		return {"mcl_fences:oak_fence_gate", 0};
	case 194:
		return {"mcl_fences:spruce_fence_gate", 0};
	case 195:
		return {"mcl_copper:block", 0};
	case 196:
		return {"mcl_panes:pane_natural_flat", 0};
	case 197:
		return {"mcl_flowers:double_fern", 0};
	case 198:
		return {"mcl_flowers:double_fern_top", 0};
	case 199:
		return {"mcl_copper:block_exposed", 0};
	case 200:
		return stair("mcl_stairs:stair_stone_rough");
	case 201:
		return {"mcl_lightning_rods:rod", 0};
	case 202:
		return {"mcl_flowerpots:flower_pot", 0};
	case 203:
		return {"mcl_ocean:sea_lantern", 0};
	case 204:
		return {"mcl_copper:block_exposed_chiseled", 0};
	case 205:
		return {"mcl_stairs:slab_warped", 0};
	case 206:
		return stair("mcl_stairs:stair_warped");
	case 207:
		return {"mcl_colorblocks:concrete_green", 0};
	case 208:
		return {"mcl_walls:brick", 0};
	case 209:
		return {"mcl_redstone_torch:redstoneblock", 0};
	case 210:
	case 362:
		return {"mcl_lanterns:chain", 0};
	case 211:
		return trapdoor("mcl_doors:trapdoor_warped", "mcl_doors:trapdoor_warped_open");
	case 212:
		return {"mcl_trees:stripped_warped", 0};
	case 213:
		return {"mcl_trees:bark_stripped_warped", 0};
	case 214:
		return {"mcl_stairs:slab_stone_double", 0};
	case 215:
		return {"mcl_copper:block_exposed_cut", 0};
	case 216:
		return {"mcl_colorblocks:hardened_clay_silver", 0};
	case 217:
		return {"mcl_stairs:slab_oak", 0};
	case 218:
		return {"mcl_redstone_lamp:lamp_on", 0};
	case 219:
		return {"mcl_trees:tree_dark_oak", 0};
	case 220:
		return {"mcl_trees:leaves_dark_oak", 0};
	case 221:
		return {"mcl_trees:tree_jungle", 0};
	case 222:
		return {"mcl_trees:leaves_jungle", 0};
	case 223:
		return {"mcl_trees:tree_acacia", 0};
	case 224:
		return {"mcl_trees:leaves_acacia", 0};
	case 225:
		return {"mcl_trees:leaves_spruce", 0};
	case 226:
		return {"mcl_core:glass_cyan", 0};
	case 227:
		return {"mcl_core:glass_blue", 0};
	case 228:
		return {"mcl_core:glass_light_blue", 0};
	case 229:
		return {"mcl_daylight_detector:daylight_detector", 0};
	case 230:
		return {"mcl_trees:tree_cherry_blossom", 0};
	case 231:
		return {"mcl_trees:leaves_cherry_blossom", 0};
	case 232:
		return {"mcl_colorblocks:concrete_powder_brown", 0};
	case 233:
		return {"mcl_trees:leaves_mangrove", 0};
	case 234:
		return {"mcl_trees:leaves_azalea", 0};
	case 235:
		return {"mcl_flowers:poppy", 0};
	case 236:
	case 299:
		return trapdoor("mcl_doors:trapdoor_oak", "mcl_doors:trapdoor_oak_open");
	case 237:
		return {"mcl_core:reeds", 0};
	case 238:
	case 239:
		return {"mcl_core:water_source", 0};
	case 240:
		return {"mcl_stairs:slab_quartzblock", 0};
	case 241:
		return trapdoor(
				"mcl_doors:trapdoor_dark_oak", "mcl_doors:trapdoor_dark_oak_open");
	case 242:
		return trapdoor("mcl_doors:trapdoor_spruce", "mcl_doors:trapdoor_spruce_open");
	case 243:
		return trapdoor("mcl_doors:trapdoor_birch", "mcl_doors:trapdoor_birch_open");
	case 244:
		return {"mcl_stairs:slab_mud_brick", 0};
	case 245:
		return {"mcl_stairs:slab_brick_block", 0};
	case 246:
		return {"mcl_flowers:tulip_red", 0};
	case 247:
		return {"mcl_flowers:dandelion", 0};
	case 248:
		return {"mcl_flowers:blue_orchid", 0};
	case 249:
		return {"mcl_core:stone_with_diamond", 0};
	case 250:
		return {"mcl_core:stone_with_redstone", 0};
	case 251:
		return {"mcl_core:stone_with_lapis", 0};
	case 252:
		return {"mcl_colorblocks:concrete_powder_grey", 0};
	case 253:
		return {"mcl_colorblocks:hardened_clay_cyan", 0};
	case 254:
		return {"mcl_wool:black", 0};
	case 255:
		return {"mcl_banners:hanging_banner", 0};
	case 256:
		return {"mcl_lever:lever_off", facedir()};
	case 257:
		return {"mcl_grindstone:grindstone", facedir()};
	case 258:
		return {"mcl_minecarts:rail", 0};
	case 259:
		return {"mcl_wool:red", 0};
	case 260:
		return {"mcl_core:ladder", facedir()};
	case 261:
		return {"mcl_wool:yellow", 0};
	case 265:
		return {"mcl_stairs:slab_cobble", 0};
	case 266:
		return {"mcl_fences:nether_brick_fence", 0};
	case 267:
		return {"mcl_fences:birch_fence", 0};
	case 268:
		return {"mcl_stairs:slab_quartz_smooth", 0};
	case 269:
		return stair("mcl_stairs:stair_quartz_smooth");
	case 270:
		return stair("mcl_stairs:stair_blackstone");
	case 271:
		return {"mcl_blackstone:wall", 0};
	case 272:
		return {"mcl_walls:diorite", 0};
	case 273:
		return {"mcl_stairs:slab_deepslate_polished", 0};
	case 274:
		return {"mcl_signs:wall_sign_oak", facedir()};
	case 275:
	case 277:
	case 278:
	case 285:
	case 317:
		return {"mcl_banners:hanging_banner", 0};
	case 276:
		return {"mcl_fences:jungle_fence", 0};
	case 279:
		return door("mcl_doors:door_birch_b_1", "mcl_doors:door_birch_t_1");
	case 280:
		return {"mcl_pressureplates:pressure_plate_birch_off", 0};
	case 281:
		return {"mcl_pressureplates:pressure_plate_stone_off", 0};
	case 282:
		return {"mcl_blast_furnace:blast_furnace", facedir()};
	case 283:
		return {"mcl_dispensers:dispenser", facedir()};
	case 284:
		return {"mcl_hoppers:hopper", 0};
	case 286:
		return {"mcl_cauldrons:cauldron_3", 0};
	case 287:
		return {"mcl_compass:lodestone", 0};
	case 288:
		return {"mcl_redstone_torch:redstone_torch_on", 0};
	case 289:
		return {"mcl_wool:red_carpet", 0};
	case 290:
		return {"mcl_blackstone:blackstone_chiseled_polished", 0};
	case 291:
		return {"mcl_walls:stonebrickmossy", 0};
	case 292:
		return stair("mcl_stairs:stair_bamboo");
	case 293:
		return door("mcl_doors:door_oak_b_1", "mcl_doors:door_oak_t_1");
	case 296:
		return {"mcl_walls:endbricks", 0};
	case 297:
		return {"mcl_stairs:slab_bamboo", 0};
	case 298:
		return {"mcl_deepslate:deepslate_chiseled", 0};
	case 300:
		return {"mcl_buttons:button_birch_off", facedir()};
	case 301:
		return {"mcl_core:cobweb", 0};
	case 302:
		return {"mcl_stairs:slab_dark_oak", 0};
	case 303:
		return {"mcl_stairs:slab_jungle", 0};
	case 304:
		return stair("mcl_stairs:stair_jungle");
	case 305:
	case 316:
	case 320:
	case 326:
		return {"mcl_books:bookshelf", 0};
	case 306:
		return {"mcl_buttons:button_oak_off", facedir()};
	case 307:
		return {"mcl_minecarts:golden_rail", 0};
	case 308:
		return {"mcl_fences:spruce_fence", 0};
	case 309:
		return {"mcl_stairs:slab_spruce", 0};
	case 310:
		return {"mcl_stairs:slab_andesite", 0};
	case 311:
		return {"mcl_stairs:slab_deepslate_cobbled", 0};
	case 312:
		return stair("mcl_stairs:stair_deepslate_cobbled");
	case 313:
		return {"mcl_fences:dark_oak_fence", 0};
	case 314:
		return {"mcl_pressureplates:pressure_plate_dark_oak_off", 0};
	case 318:
		return {"mcl_wool:grey", 0};
	case 319:
		return {"mcl_nether:nether_wart_block", 0};
	case 321:
		return {"mcl_blackstone:basalt_polished", 0};
	case 322:
		return {"mcl_buttons:button_polished_blackstone_off", facedir()};
	case 323:
		return {"mcl_pressureplates:pressure_plate_polished_blackstone_off", 0};
	case 324:
		return {"mcl_stairs:slab_red_nether_brick", 0};
	case 325:
		return {"mcl_buttons:button_spruce_off", facedir()};
	case 327:
		return trapdoor("mcl_doors:trapdoor_acacia", "mcl_doors:trapdoor_acacia_open");
	case 328:
		return {"mcl_composters:composter", 0};
	case 329:
		return {"mcl_wool:cyan_carpet", 0};
	case 330:
		return {"mcl_buttons:button_dark_oak_off", facedir()};
	case 331:
		return {"mcl_stairs:slab_end_bricks", 0};
	case 332:
		return {"mcl_anvils:anvil_damage_2", 0};
	case 333:
		return {"mcl_wool:green_carpet", 0};
	case 334:
		return {"mcl_wool:light_blue_carpet", 0};
	case 335:
		return {"mcl_walls:netherbrick", 0};
	case 336:
		return {"mcl_smoker:smoker", facedir()};
	case 337:
		return {"mcl_core:redsandstonesmooth2", 0};
	case 338:
		return {"mcl_stairs:slab_redsandstonesmooth2", 0};
	case 339:
		return {"mcl_panes:pane_blue_flat", 0};
	case 340:
		return {"mcl_wool:cyan", 0};
	case 341:
		return {"mcl_wool:silver_carpet", 0};
	case 342:
		return {"mcl_stairs:slab_mossycobble", 0};
	case 343:
		return {"mcl_stairs:slab_stonebrickmossy", 0};
	case 344:
		return {"mcl_ocean:prismarine", 0};
	case 345:
		return {"mcl_end:end_rod", facedir()};
	case 347:
		return {"mcl_signs:wall_sign_spruce", facedir()};
	case 348:
		return stair("mcl_stairs:stair_granite");
	case 349:
		return stair("mcl_stairs:stair_diorite");
	case 350:
		return {"mcl_deepslate:deepslate_tiles", 0};
	case 351:
		return {"mcl_stairs:slab_deepslate_tiles", 0};
	case 352:
		return {"mcl_deepslate:deepslatetileswall", 0};
	case 353:
		return {"mcl_stairs:slab_blackstone_polished", 0};
	case 354:
		return {"mcl_stairs:slab_diorite_smooth", 0};
	case 355:
		return {"mcl_lanterns:soul_lantern_floor", 0};
	case 356:
		return {"mcl_nether:quartz_chiseled", 0};
	case 357:
		return {"mcl_nether:quartz_pillar", 0};
	case 358:
		return {"mcl_redstone_torch:redstone_torch_on_wall", facedir()};
	case 359:
		return {"mcl_core:goldblock", 0};
	case 360:
		return {"mcl_wool:orange", 0};
	case 361:
		return {"mcl_wool:blue", 0};
	case 363:
		return {"mcl_banners:hanging_banner", 0};
	case 364:
	case 365:
		return door("mcl_doors:door_spruce_b_1", "mcl_doors:door_spruce_t_1");
	case 366:
		return door("mcl_doors:door_oak_b_1", "mcl_doors:door_oak_t_1");
	case 367:
		return {"mcl_end:end_stone", 0};
	case 368:
		return {"mcl_end:purpur_block", 0};
	case 369:
		return {"mcl_stairs:slab_purpur_block", 0};
	case 370:
		return stair("mcl_stairs:stair_purpur_block");
	case 371:
		return {"mcl_trees:wood_crimson", 0};
	case 372:
		return {"mcl_stairs:slab_crimson", 0};
	case 373:
		return stair("mcl_stairs:stair_crimson");
	case 374:
		return {"mcl_trees:wood_cherry_blossom", 0};
	case 375:
		return {"mcl_stairs:slab_cherry_blossom", 0};
	case 376:
		return stair("mcl_stairs:stair_cherry_blossom");
	case 377:
		return {"mcl_ocean:prismarine_dark", 0};
	case 378:
		return {"mcl_stairs:slab_prismarine_dark", 0};
	case 379:
		return stair("mcl_stairs:stair_prismarine_dark");
	case 380:
		return {"mcl_stairs:slab_copper_exposed_cut", 0};
	case 381:
		return trapdoor(
				"mcl_doors:trapdoor_pale_oak", "mcl_doors:trapdoor_pale_oak_open");
	case 382:
		return {"mcl_core:coalblock", 0};
	case 383:
		return {"mcl_stairs:slab_blackstone", 0};
	case 384:
		return door("mcl_doors:iron_door_b_1", "mcl_doors:iron_door_t_1");
	default:
		return {"mcl_core:stone", 0};
	}
}
}
