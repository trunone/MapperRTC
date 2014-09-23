// -*-C++-*-
/*!
 * @file  MapServiceSVC_impl.h
 * @brief Service implementation header of MapService.idl
 *
 */

#include "MapServiceSkel.h"

#ifndef MAPSERVICESVC_IMPL_H
#define MAPSERVICESVC_IMPL_H
 
/*!
 * @class SimpleMapSVC_impl
 * Example class implementing IDL interface Map::SimpleMap
 */
class SimpleMapSVC_impl
 : public virtual POA_Map::SimpleMap,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~SimpleMapSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   SimpleMapSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~SimpleMapSVC_impl();

   // attributes and operations
   void getMap(CORBA::String_out arg);

};



#endif // MAPSERVICESVC_IMPL_H


