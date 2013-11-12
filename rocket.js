function assert(condition, message) {
    if (!condition) {
	throw message || "Assertion failed";
    }
}

function init() {
    var SCALE = 3000
    ,   GRAV = 6.674e-11 * SCALE
    ;
    
    var   b2Vec2 = Box2D.Common.Math.b2Vec2
    ,     b2AABB = Box2D.Collision.b2AABB
    ,	  b2BodyDef = Box2D.Dynamics.b2BodyDef
    ,	  b2Body = Box2D.Dynamics.b2Body
    ,	  b2FixtureDef = Box2D.Dynamics.b2FixtureDef
    ,	  b2Fixture = Box2D.Dynamics.b2Fixture
    ,	  b2World = Box2D.Dynamics.b2World
    ,	  b2MassData = Box2D.Collision.Shapes.b2MassData
    ,	  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
    ,	  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
    ,	  b2DebugDraw = Box2D.Dynamics.b2DebugDraw
    ,     b2MouseJointDef =  Box2D.Dynamics.Joints.b2MouseJointDef
    ;

    // Create the world without gravity
    var world = new b2World(new b2Vec2(0, 0),
			    true);
    
    var planets = [];
    function addPlanet(x, y, radius, mass) {
	assert(arguments.length == 4, "Not enough arguments for addPlanet");

	var fixDef = new b2FixtureDef;
	fixDef.density = mass / (4.0 / 3.0 * Math.PI * Math.exp(radius / SCALE, 3));
	fixDef.friction = 0.5;
	fixDef.restitution = 0.2;
	
	var bodyDef = new b2BodyDef;
	bodyDef.type = b2Body.b2_dynamicBody;
	bodyDef.position.x = x / SCALE;
	bodyDef.position.y = y / SCALE;
	
	fixDef.shape = new b2CircleShape(radius / SCALE);
	var body = world.CreateBody(bodyDef);
	planets.push({'mass': mass, 'body': body});
	body.CreateFixture(fixDef);
    }
    
    addPlanet(SCALE * 200, 50, 4000, 50);
    addPlanet(SCALE * 400, 50, 4000, 50);
    
    console.log(planets);

    // Set up debugDraw
    var debugDraw = new b2DebugDraw();
    debugDraw.SetSprite(document.getElementById("canvas").getContext("2d"));
    debugDraw.SetDrawScale(SCALE);
    debugDraw.SetFillAlpha(0.3);
    debugDraw.SetLineThickness(1.0);
    debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
    world.SetDebugDraw(debugDraw);

    // Start updating
    (function update() {
	world.Step(1/60,
		   2,
		   2);
	world.DrawDebugData();
	world.ClearForces();
	
	// Simulate gravity
	for (var i=0; i<planets.length; i++) {
	    var planetDef = planets[i];
	    for (var j=i+1; j<planets.length; j++) {
		var planet2Def = planets[j];
		var from1to2 = new b2Vec2(0, 0);
		from1to2.Add(planet2Def.body.GetWorldCenter());
		from1to2.Subtract(planetDef.body.GetWorldCenter());
		
		var distance = from1to2.Length();
		from1to2.Multiply(1 / distance);
		
		var forceMagnitude = GRAV * planetDef.mass * planet2Def.mass / Math.exp(distance, 2);
		
		from1to2.Multiply(forceMagnitude);
		
		planetDef.body.ApplyForce(from1to2, planetDef.body.GetWorldCenter());
		from1to2.Multiply(-1);
		planet2Def.body.ApplyForce(from1to2, planet2Def.body.GetWorldCenter());
	    }
	}
	
	requestAnimationFrame(update);
    })();
    
    
}