#include "startend_translator.h"
#include "translator.h"
namespace arnis::map_transformation
{
void StartEndTranslator::operate(
		std::vector<ProcessedElement> &e, cartesian::XZBBox &b, Ground &)
{
	translate_by_vector(displacement(), e, b);
}
}
