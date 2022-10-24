use futures::TryStreamExt;
use mongodb::{
    bson::{doc, Document},
    Client, Collection,
};
use serde::Deserialize;

use async_graphql::{http::GraphiQLSource, Schema};
use async_graphql_poem::{GraphQL, GraphQLSubscription};
use books::{MutationRoot, QueryRoot, Storage, SubscriptionRoot};
use poem::{
    get, handler,
    listener::TcpListener,
    middleware::AddData,
    web::{Data, Html, Json},
    EndpointExt, IntoResponse, Route, Server,
};

#[handler]
async fn graphiql() -> impl IntoResponse {
    Html(
        GraphiQLSource::build()
            .endpoint("http://localhost:4000")
            .subscription_endpoint("ws://localhost:4000/ws")
            .finish(),
    )
}

#[handler]
async fn get_users(collection: Data<&Collection<Document>>) -> Json<serde_json::Value> {
    let cursor = collection.find(None, None).await.unwrap();
    let result = cursor.try_collect::<Vec<Document>>().await.unwrap();

    Json(serde_json::json!(result))
}

#[derive(Deserialize)]
struct InsertableUser {
    name: String,
    email: String,
    age: u32,
}

#[handler]
async fn create_user(
    collection: Data<&Collection<Document>>,
    req: Json<InsertableUser>,
) -> Json<serde_json::Value> {
    let result = collection
        .insert_one(
            doc! {
                "name": &req.name,
                "email": &req.email,
                "age": req.age
            },
            None,
        )
        .await
        .unwrap();
    let result = collection
        .find_one(doc! {"_id": result.inserted_id}, None)
        .await
        .unwrap();

    Json(serde_json::json!(result))
}

#[tokio::main]
async fn main() {
    if std::env::var_os("RUST_LOG").is_none() {
        std::env::set_var("RUST_LOG", "poem=debug");
    }
    tracing_subscriber::fmt::init();

    // 替换 mongodb 用户名和密码！
    let mongodb = Client::with_uri_str("mongodb+srv://<username>:<password>@cluster0.iumxjhi.mongodb.net/?retryWrites=true&w=majority")
        .await
        .unwrap()
        .database("test");
    let collection = mongodb.collection::<Document>("user");

    let schema = Schema::build(QueryRoot, MutationRoot, SubscriptionRoot)
        .data(Storage::default())
        .finish();

    let app = Route::new()
        .at("/", get(graphiql).post(GraphQL::new(schema.clone())))
        .at("/ws", get(GraphQLSubscription::new(schema)))
        .at("/user", get(get_users).post(create_user))
        .with(AddData::new(collection));

    println!("GraphiQL IDE: http://localhost:4000");
    Server::new(TcpListener::bind("0.0.0.0:4000"))
        .run(app)
        .await
        .unwrap();
    // mutation {
    //   createBook(name: "name", author: "author")
    // }
}
