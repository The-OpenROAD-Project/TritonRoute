#define BOOST_TEST_MODULE gc
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#include "frDesign.h"
#include "gc/FlexGC.h"

using namespace fr;

// BOOST_TEST wants an operator<< for any type it compares.  We
// don't have those for enums and they are tedious to write.
// Just compare them as integers to avoid this requirement.
#define TEST_ENUM_EQUAL(L, R) \
  BOOST_TEST(static_cast<int>(L) == static_cast<int>(R))

/////////
// Various test helpers to build up the db.
/////////

namespace {

void add_layer(frTechObject* tech,
               const char* name,
               frLayerTypeEnum type,
               frPrefRoutingDirEnum dir = frcNonePrefRoutingDir)
{
  auto layer = std::make_unique<frLayer>();
  layer->setLayerNum(tech->getTopLayerNum() + 1);
  layer->setName(name);
  layer->setType(type);
  layer->setDir(dir);

  layer->setWidth(100);
  layer->setMinWidth(100);
  layer->setPitch(200);

  // These constraints are mandatory
  if (type == frLayerTypeEnum::ROUTING) {
    auto minWidthConstraint
        = std::make_unique<frMinWidthConstraint>(layer->getMinWidth());
    layer->setMinWidthConstraint(minWidthConstraint.get());
    tech->addUConstraint(std::move(minWidthConstraint));

    auto offGridConstraint = std::make_unique<frOffGridConstraint>();
    layer->setOffGridConstraint(offGridConstraint.get());
    tech->addUConstraint(std::move(offGridConstraint));
  }

  auto shortConstraint = std::make_unique<frShortConstraint>();
  layer->setShortConstraint(shortConstraint.get());
  tech->addUConstraint(std::move(shortConstraint));

  tech->addLayer(std::move(layer));
}

void setup_tech(frTechObject* tech)
{
  tech->setManufacturingGrid(10);
  tech->setDBUPerUU(1000);

  // TR assumes that masterslice always exists
  add_layer(tech, "masterslice", frLayerTypeEnum::MASTERSLICE);
  add_layer(tech, "v0", frLayerTypeEnum::CUT);
  add_layer(tech, "m1", frLayerTypeEnum::ROUTING);
}

std::unique_ptr<frDesign> make_design()
{
  auto design = std::make_unique<frDesign>();
  setup_tech(design->getTech());

  auto block = std::make_unique<frBlock>();
  block->setName("test");

  // GC assumes these fake nets exist
  auto vssFakeNet = std::make_unique<frNet>("frFakeVSS");
  vssFakeNet->setType(frNetEnum::frcGroundNet);
  vssFakeNet->setIsFake(true);
  block->addFakeSNet(std::move(vssFakeNet));

  auto vddFakeNet = std::make_unique<frNet>("frFakeVDD");
  vddFakeNet->setType(frNetEnum::frcPowerNet);
  vddFakeNet->setIsFake(true);
  block->addFakeSNet(std::move(vddFakeNet));

  design->setTopBlock(std::move(block));

  return design;
}

frNet* make_net(frBlock* block, const char* name)
{
  auto net_p = std::make_unique<frNet>(name);
  frNet* net = net_p.get();
  block->addNet(std::move(net_p));
  return net;
}

void make_pathseg(frNet* net,
                  frLayerNum layer_num,
                  const frPoint& begin,
                  const frPoint& end,
                  frUInt4 width = 100,
                  frEndStyleEnum begin_style = frcTruncateEndStyle,
                  frEndStyleEnum end_style = frcTruncateEndStyle)
{
  auto ps = std::make_unique<frPathSeg>();
  ps->setPoints(begin, end);
  ps->setLayerNum(layer_num);

  frSegStyle style;
  style.setWidth(width);
  style.setBeginStyle(begin_style);
  style.setEndStyle(end_style);

  ps->setStyle(style);
  net->addShape(std::move(ps));
}

void init_region_query(frDesign* design)
{
  int num_layers = design->getTech()->getLayers().size();

  frRegionQuery* query = design->getRegionQuery();
  query->init(num_layers);
  query->initDRObj(num_layers);
}

void test_marker(frMarker* marker,
                 frLayerNum layer_num,
                 frConstraintTypeEnum type,
                 const frBox& expected_bbox)
{
  frBox bbox;
  marker->getBBox(bbox);

  BOOST_TEST(marker->getLayerNum() == layer_num);
  BOOST_TEST(marker->getConstraint());
  TEST_ENUM_EQUAL(marker->getConstraint()->typeId(), type);
  BOOST_TEST(bbox == expected_bbox);
}

}  // end anonymous namespace

BOOST_AUTO_TEST_SUITE(gc);

// Two touching metal shape from different nets generate a short
BOOST_AUTO_TEST_CASE(metal_short)
{
  // Setup
  auto design = make_design();

  frBlock* block = design->getTopBlock();

  frNet* n1 = make_net(block, "n1");
  frNet* n2 = make_net(block, "n2");

  make_pathseg(n1, 2, {0, 0}, {500, 0});
  make_pathseg(n2, 2, {500, 0}, {1000, 0});

  init_region_query(design.get());

  // Run the GC engine
  const frBox work(0, 0, 2000, 2000);

  FlexGCWorker worker(design.get());
  worker.setExtBox(work);
  worker.setDrcBox(work);

  worker.init();
  worker.main();
  worker.end();

  // Test the results
  auto& markers = worker.getMarkers();
  BOOST_TEST(markers.size() == 1);

  test_marker(markers[0].get(),
              2,
              frConstraintTypeEnum::frcShortConstraint,
              frBox(500, -50, 500, 50));
}

BOOST_AUTO_TEST_SUITE_END();
