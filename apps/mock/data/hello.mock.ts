export const ApiGetHello = {
  url: 'hello',
  method: 'get',
  response: (req: any) => {
    return {
      hello: 'helllo world',
      body: req?.body,
      query: req?.query,
    }
  },
}
