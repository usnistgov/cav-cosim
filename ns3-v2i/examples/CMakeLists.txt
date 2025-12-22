build_lib_example(
  NAME ns3_gateway_v2i
  SOURCE_FILES ns3_gateway_v2i.cc
  LIBRARIES_TO_LINK
        ${libapplications}
        ${libcore}
        ${libcsma}
        ${libinternet}
        ${libmobility}
        ${libnetwork}
        ${libwifi}
        ${libns3-v2i}
)
build_lib_example(
  NAME ns3_gateway_v2i_sl_nr_text
  SOURCE_FILES ns3_gateway_v2i_sl_nr_text.cc
  LIBRARIES_TO_LINK
        ${libapplications}
        ${libcore}
        ${libcsma}
        ${libinternet}
        ${libmobility}
        ${libnetwork}
        ${libnr}
        ${libns3-v2i}
)
build_lib_example(
  NAME ns3_gateway_v2i_sl_nr_json
  SOURCE_FILES ns3_gateway_v2i_sl_nr_json.cc
  LIBRARIES_TO_LINK
        ${libapplications}
        ${libcore}
        ${libcsma}
        ${libinternet}
        ${libmobility}
        ${libnetwork}
        ${libnr}
        ${libns3-v2i}
)
