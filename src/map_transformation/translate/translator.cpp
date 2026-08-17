#include "translator.h"
namespace arnis::map_transformation
{
void translate_by_vector(
		cartesian::XZVector v, std::vector<ProcessedElement> &es, cartesian::XZBBox &b)
{
	b += v;
	for (auto &e : es) {
		if (e.is_node()) {
			auto &n = std::get<ProcessedNode>(e);
			n.x += v.dx;
			n.z += v.dz;
		} else if (e.is_way()) {
			auto &w = std::get<ProcessedWay>(e);
			for (auto &n : w.nodes) {
				n.x += v.dx;
				n.z += v.dz;
			}
		} else if (e.is_relation()) {
			auto &relation = std::get<ProcessedRelation>(e);
			for (auto &member : relation.members)
				for (auto &n : member.way.nodes) {
					n.x += v.dx;
					n.z += v.dz;
				}
		}
	}
}
}
