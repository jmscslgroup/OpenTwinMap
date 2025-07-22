class XODRWriter:
    osm_to_xodr_parser = None
    xodr_path = None

    def __init__(self, osm_to_xodr_parser, xodr_path):
        self.osm_to_xodr_parser = osm_to_xodr_parser
        self.xodr_path = xodr_path

    def startBasicXODRFile(self):
        #referenceLon, referenceLat, topoParameter = giveReferences()
        #xmin, xmax, ymin, ymax = topoParameter
        reference_lon, reference_lat = self.osm_to_xodr_parser.getOriginCoordinates()
        xmin, ymin, xmax, ymax = self.osm_to_xodr_parser.getBoundsInMeters()
        with open(path,'w',encoding='utf-8') as f:
            f.write('''<?xml version="1.0" encoding="UTF-8"?>
        <OpenDRIVE>
        <header revMajor="1" revMinor="4" name="" version="1" date="2019-02-18T13:36:12" north="{0}" south="{1}" east="{2}" west="{3}">
        <geoReference><![CDATA[+proj=tmerc +lat_0={4:0.10f} +lon_0={5:0.10f} +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs]]></geoReference>
        </header>
        <!-- Roads -->
        <!-- nextRoad -->
        <!-- Junctions -->
        <!-- nextJunction -->
        </OpenDRIVE>
        '''.format(ymax-ymin, 0.0, xmax-xmin, 0.0, reference_lat, reference_lon))

    def fillNormalRoads(self):
        filedata = ""
        with open(self.xodr_path,'r',encoding='utf-8') as file:
            filedata = file.read()
        parts = filedata.split("<!-- nextRoad -->")
        for road in self.osm_to_xodr_parser.all_ways.values():
            name = "Road "+ str(road.xodr_id)
            try: name = road.tags["name"]
            except: pass
            maxspeed = "30"
            try: maxspeed = road.tags["maxspeed"]
            except: pass
            #add road string

            # create geometry
            geometry = ""
            lengths = []
            for element in road.road_elements:
                lengths.append(element["length"])
                geometry += '''
                <geometry s="{0}" x="{1}" y="{2}" hdg="{3}" length="{4}">'''.format(sum(lengths[:-1]), element["xstart"],
                                                                                element["ystart"], element["heading"],
                                                                                element["length"])+('''
                    <line/>''' if element["curvature"] == 0.0 else '''
                    <arc curvature="{0}"/>'''.format(element["curvature"])) + '''
                </geometry>'''
            lengths = []
            elevation = ""
            # create elevation
            for element in road.elevation_elements:
                lengths.append(element["length"])
                elevation += '''
                <elevation s="{0}" a="{1}" b="{2}" c="0.0" d="0.0"/>'''.format(sum(lengths[:-1]),element["zstart"], element["steigung"])

            
            leftlanes = ""
            leftlanenumber = 1
            for i in range(road.lane_number_opposite):
                leftlanes += '''
                            <lane id="{0}" type="driving" level="false">
                                            <link>
                                            </link>
                                            <width sOffset="0.0" a="4.00e+00" b="0.0" c="0.00" d="0.00"/>
                                            <roadMark sOffset="0.00" type="{1}" material="standard" color="white" laneChange="none"/>
                            </lane>'''.format(leftlanenumber, "solid" if leftlanenumber == road.lane_number_opposite else "broken")
                leftlanenumber += 1
            rightlanes = ""
            rightlanenumber = -1
            for i in range(road.lane_number_direction):
                rightlanes += '''
                            <lane id="{0}" type="driving" level="false">
                                            <link>
                                            </link>
                                            <width sOffset="0.0" a="4.00e+00" b="0.0" c="0.00" d="0.00"/>
                                            <roadMark sOffset="0.00" type="{1}" material="standard" color="white" laneChange="none"/>
                            </lane>'''.format(rightlanenumber, "solid" if rightlanenumber == -road.lane_number_direction else "broken")
                rightlanenumber -= 1

            parts[0] +='''
            <road name="{0}" length="{1}" id="{2}" junction="-1">
                <link>
                    <predecessor elementType="junction" elementId="{3}"/>
                    <successor elementType="junction" elementId="{4}"/>
                </link>'''.format(name, sum(lengths), road.xodr_id, road.start_junction, road.end_junction)+'''
            <type s="0.0" type="town">
                <speed max="{0}" unit="mph"/>
            </type>
                <planView>'''.format(maxspeed) + geometry +'''
                </planView>

            <elevationProfile>''' + elevation + '''
            </elevationProfile>
                <lanes>
                    <laneOffset s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
                    <laneSection s="0.0">
                        <left>'''+leftlanes+'''
                        </left>
                        <center>
                            <lane id="0" type="none" level="false">
                                <roadMark sOffset="0.00" type="{0}" material="standard" color="white" width="1.2500000000000000e-1" laneChange="none"/>
                            </lane>
                        </center>
                        <right>'''.format("broken" if (road.lane_number_opposite == 1 and road.lane_number_direction == 1) else "solid")+rightlanes+'''
                        </right>
                    </laneSection>
                </lanes>
            </road>
            '''
        with open(self.xodr_path,'w',encoding='utf-8') as f:
            f.write("<!-- nextRoad -->".join(parts))

    def fillJunctionRoads(self):
        filedata = ""
        with open(self.xodr_path,'r',encoding='utf-8') as file:
            filedata = file.read()
        parts = filedata.split("<!-- nextRoad -->")
        secondsplits = parts[1].split("<!-- nextJunction -->")
        parts[1] = secondsplits[0]
        parts.append(secondsplits[1])
        for junction in self.osm_to_xodr_parser.junction_nodes.keys():
            # create junction start
            parts[1] += '''
            <junction id="{0}" name="{1}">'''.format(str(junction),"junction "+str(junction))
            connection_id = 1
            for roadkey in self.osm_to_xodr_parser.junction_nodes[junction].keys():
                incoming_road,outgoing_road = roadkey.split("_to_")
                for lanekey in self.osm_to_xodr_parser.junction_nodes[junction][roadkey].keys():
                        from_lane,to_lane = lanekey.split("_to_")
                        road = self.osm_to_xodr_parser.junction_nodes[junction][roadkey][lanekey]
                        #create connection
                        parts[1] += '''
                        <connection id="{0}" incomingRoad="{1}" connectingRoad="{2}" contactPoint="{3}">
                            <laneLink from="{4}" to="{5}"/>
                        </connection>'''.format(connection_id, incoming_road, road.xodr_id, "start",
                                            from_lane, "-1")
                        connection_id +=1

                        #create road
                        geometry = ""
                        lengths = []
                        for element in road.roadElements:
                            lengths.append(element["length"])
                            geometry += '''
                            <geometry s="{0}" x="{1}" y="{2}" hdg="{3}" length="{4}">'''.format(sum(lengths[:-1]), element["xstart"],
                                                                                            element["ystart"], element["heading"],
                                                                                            element["length"])+('''
                                <line/>''' if element["curvature"] == 0.0 else '''
                                <arc curvature="{0}"/>'''.format(element["curvature"])) + '''
                            </geometry>'''

                        lengths = []
                        elevation = ""
                        # create elevation
                        for element in road.elevationElements:
                            lengths.append(element["length"])
                            elevation += '''
                            <elevation s="{0}" a="{1}" b="{2}" c="0.0" d="0.0"/>'''.format(sum(lengths[:-1]),element["zstart"], element["steigung"])

                        name = "JunctionConnection "+ roadkey + " lane "+lanekey
                        maxspeed = "30"
                        parts[0] +='''
            <road name="{0}" length="{1}" id="{2}" junction="{3}">
                <link>
                    <predecessor elementType="road" elementId="{4}" contactPoint="{6}"/>
                    <successor elementType="road" elementId="{5}" contactPoint="{7}"/>
                </link>'''.format(name, sum(lengths), road.xodr_id, junction, incoming_road, outgoing_road,
                                road.contact_point_predecessor, road.contact_point_successor)+'''
            <type s="0.0" type="town">
                <speed max="{0}" unit="mph"/>
            </type>
                <planView>'''.format(maxspeed) + geometry +'''
                </planView>

            <elevationProfile>''' + elevation + '''
            </elevationProfile>
                <lanes>
                    <laneOffset s="0.0" a="{0}" b="{1}" c="0.0" d="0.0"/>'''.format(road.lane_offset_a, road.lane_offset_b) + '''
                    <laneSection s="0.0">
                        <center>
                            <lane id="0" type="none" level="false">
                                <roadMark sOffset="0.0000000000000000e+0" type="none" material="standard" color="yellow" width="1.2500000000000000e-1" laneChange="none"/>
                            </lane>
                        </center>
                        <right>
                            <lane id="-1" type="driving" level="false">
                                <link>
                                    <predecessor id="{0}"/>
                                    <successor id="{1}"/>
                                </link>
                                <width sOffset="0.0" a="4.00e+00" b="0.0" c="0.00" d="0.00"/>
                                <roadMark sOffset="0.00" type="none" material="standard" color="white" laneChange="none"/>
                            </lane>
                        </right>
                    </laneSection>
                </lanes>
            </road>
            '''.format(from_lane,to_lane)
            #close junction
            parts[1] += '''
            </junction>
            '''
        parts[0] = "<!-- nextRoad -->".join([parts[0],parts[1]])
        whole = "<!-- nextJunction -->".join([parts[0],parts[2]])

        with open(self.xodr_path,'w',encoding='utf-8') as f:
                f.write(whole)
