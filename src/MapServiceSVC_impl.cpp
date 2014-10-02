// -*-C++-*-
/*!
 * @file  MapServiceSVC_impl.cpp
 * @brief Service implementation code of MapService.idl
 *
 */

#include "MapServiceSVC_impl.h"
#include "MapBuilder.h"

/*
 * Example implementational code for IDL interface Map::SimpleMap
 */
SimpleMapSVC_impl::SimpleMapSVC_impl()
{
  // Please add extra constructor code here.
}


SimpleMapSVC_impl::~SimpleMapSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void SimpleMapSVC_impl::getMap(CORBA::String_out arg)
{
    CSimpleMap map;
    MapBuilder::get_instance()->get_map(map);

    //std::string str = ObjectToString(new CSerializable(map));

    //arg = CORBA::string_dup(str.c_str());
    arg = CORBA::string_dup("Test");
}



// End of example implementational code



