#include "translator_factory.h"
#include "vector_translator.h"
#include "startend_translator.h"
#include <sstream>
namespace arnis::map_transformation
{
std::unique_ptr<Operator> translator_from_json(
		const std::string &type, const std::string &config)
{
	std::stringstream s(config);
	int a, b, c, d;
	if (type == "vector" && s >> a >> b) {
		auto p = std::make_unique<VectorTranslator>();
		p->vector = {a, b};
		return p;
	}
	if (type == "startend" && s >> a >> b >> c >> d) {
		auto p = std::make_unique<StartEndTranslator>();
		p->start = {a, b};
		p->end = {c, d};
		return p;
	}
	return {};
}
}
